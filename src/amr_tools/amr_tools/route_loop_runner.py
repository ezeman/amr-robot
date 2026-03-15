import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


@dataclass
class RoutePoint:
    x: float
    y: float
    yaw: float  # radians


def _yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def load_route(route_path: str, fallback_frame: str) -> tuple[list[RoutePoint], str]:
    with open(route_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    frame_id = str(data.get('frame_id', fallback_frame))
    waypoints = data.get('waypoints')
    if not waypoints:
        raise ValueError('No waypoints found in route file.')

    route: List[RoutePoint] = []
    for idx, wp in enumerate(waypoints):
        try:
            x = float(wp['x'])
            y = float(wp['y'])
        except Exception as exc:
            raise ValueError(f'Waypoint {idx} missing x/y: {exc}') from exc

        yaw = wp.get('yaw', 0.0)
        if 'yaw_deg' in wp:
            yaw = math.radians(float(wp['yaw_deg']))
        else:
            yaw = float(yaw)

        route.append(RoutePoint(x=x, y=y, yaw=yaw))

    return route, frame_id


_LIFECYCLE_STATE_LABEL = {
    0: 'unknown',
    1: 'unconfigured',
    2: 'inactive',
    3: 'active',
    4: 'finalized',
}

_GOAL_STATUS_LABEL = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class RouteLoopRunner(Node):
    """Send NavigateThroughPoses goals for AGV-style route testing."""

    def __init__(
        self,
        route: List[RoutePoint],
        frame_id: str,
        loop: bool,
        pause_sec: float,
        yaw_mode: str,
        base_frame: str,
        start_mode: str,
        start_index: int,
        start_tf_timeout_sec: float,
        reverse: bool,
        roundtrips: int,
        stop_on_failure: bool,
    ):
        super().__init__('route_loop_runner')
        self._route_forward = route
        self._route_reverse = list(reversed(route))
        self._route = self._route_reverse if reverse else self._route_forward
        self._route_active = self._route
        self._frame_id = frame_id
        self._loop = loop
        self._pause_sec = pause_sec
        self._yaw_mode = yaw_mode
        self._base_frame = base_frame
        self._start_mode = start_mode
        self._start_index = int(start_index)
        self._start_tf_timeout_sec = float(start_tf_timeout_sec)
        self._reverse = bool(reverse)
        self._stop_on_failure = bool(stop_on_failure)
        self._legs_remaining = max(int(roundtrips), 0) * 2
        self._alternate_enabled = self._legs_remaining > 0
        self._direction = 'reverse' if self._reverse else 'forward'
        self._shutdown = False
        self._goal_handle = None
        self._loop_timer = None
        self._client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._startup_timer = self.create_timer(0.5, self._maybe_start)
        self._last_cmd_vel: Optional[Twist] = None
        self._last_cmd_vel_time = 0.0
        self._last_cmd_vel_nav: Optional[Twist] = None
        self._last_cmd_vel_nav_time = 0.0
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_nav_cb, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = time.monotonic()

    def _cmd_vel_nav_cb(self, msg: Twist):
        self._last_cmd_vel_nav = msg
        self._last_cmd_vel_nav_time = time.monotonic()

    def _select_start_index_nearest(self) -> Optional[int]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=max(self._start_tf_timeout_sec, 0.1)),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"TF lookup failed for {self._frame_id} -> {self._base_frame}: {exc}. "
                "Localization is likely not ready yet (no map->odom).",
                throttle_duration_sec=2.0,
            )
            return None

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)

        best_i = 0
        best_d = float('inf')
        for i, wp in enumerate(self._route):
            d = math.hypot(wp.x - x, wp.y - y)
            if d < best_d:
                best_d = d
                best_i = i
        self.get_logger().info(
            f"Start selection: nearest waypoint index={best_i} distance={best_d:.2f} m "
            f"(robot {self._frame_id} x={x:.2f} y={y:.2f})"
        )
        return best_i

    def _prepare_active_route(self):
        # Ensure active route uses the same ordering as the base route.
        self._route_active = self._route

        if self._start_mode == 'fixed':
            idx = max(0, min(self._start_index, len(self._route_active) - 1))
        elif self._start_mode == 'nearest':
            nearest = self._select_start_index_nearest()
            if nearest is None:
                raise RuntimeError('TF not ready for nearest start selection.')
            idx = nearest
        else:
            idx = 0

        if idx == 0:
            return

        self._route_active = self._route_active[idx:]
        self.get_logger().info(f"Using route starting at index {idx}: {len(self._route_active)} poses")

    def _set_direction(self, direction: str) -> None:
        if direction not in ('forward', 'reverse'):
            raise ValueError(f'Invalid direction: {direction}')
        self._direction = direction
        self._route = self._route_forward if direction == 'forward' else self._route_reverse

    def _yaw_for_waypoint(self, index: int) -> float:
        if self._yaw_mode == 'zero':
            return 0.0
        if self._yaw_mode == 'from_file':
            return self._route_active[index].yaw
        if self._yaw_mode == 'heading':
            if index < len(self._route_active) - 1:
                curr = self._route_active[index]
                nxt = self._route_active[index + 1]
                return math.atan2(nxt.y - curr.y, nxt.x - curr.x)
            if len(self._route_active) >= 2:
                prev = self._route_active[-2]
                curr = self._route_active[-1]
                return math.atan2(curr.y - prev.y, curr.x - prev.x)
            return self._route_active[index].yaw
        return self._route_active[index].yaw

    def _maybe_start(self):
        if not self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for navigate_through_poses action server...', throttle_duration_sec=5.0)
            return

        # Ensure TF is connected before sending a map-frame route.
        if self._frame_id == 'map':
            try:
                self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    Time(),
                    timeout=Duration(seconds=max(self._start_tf_timeout_sec, 0.1)),
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                self.get_logger().warn(
                    f"Waiting for TF {self._frame_id} -> {self._base_frame} ({exc}). "
                    "Set AMCL initial pose (RViz2 2D Pose Estimate or `ros2 run amr_tools set_initial_pose ...`).",
                    throttle_duration_sec=2.0,
                )
                return

        self._startup_timer.cancel()
        try:
            self._prepare_active_route()
        except Exception as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        self.get_logger().info('Action server is ready. Sending initial route.')
        self._send_goal()

    def _stamped_poses(self) -> List[PoseStamped]:
        now = self.get_clock().now().to_msg()
        poses: List[PoseStamped] = []
        for idx, wp in enumerate(self._route_active):
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = now
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            yaw = self._yaw_for_waypoint(idx)
            qx, qy, qz, qw = _yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)
        return poses

    def _send_goal(self):
        if self._shutdown:
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._stamped_poses()
        goal_msg.behavior_tree = ''

        extras: list[str] = []
        if self._alternate_enabled:
            extras.append(f"legs_remaining={self._legs_remaining}")
            extras.append(f"direction={self._direction}")
        elif self._reverse:
            extras.append("direction=reverse")
        extra_str = f", {', '.join(extras)}" if extras else ""
        self.get_logger().info(f"Sending {len(goal_msg.poses)} poses (loop={self._loop}{extra_str})")
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Route goal was rejected.')
            if self._loop and not self._shutdown:
                self.get_logger().info(f'Retrying in {self._pause_sec} seconds...')
                self._schedule_next()
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Route goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status if result else GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Route finished successfully.')
        else:
            label = _GOAL_STATUS_LABEL.get(status, str(status))
            self.get_logger().warn(f'Route finished with status {status} ({label}).')
            if status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(
                    'Hints: check Nav2 log for "detected collision ahead" or "Failed to make progress". '
                    'Verify `/velocity_smoother` is active and `/cmd_vel` is being published.'
                )

        if self._shutdown:
            return

        if status != GoalStatus.STATUS_SUCCEEDED and self._stop_on_failure:
            self.get_logger().error('Stopping after failure (stop_on_failure=true).')
            return

        if self._alternate_enabled:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._legs_remaining -= 1
                if self._legs_remaining <= 0:
                    self.get_logger().info('All roundtrips completed.')
                    return
            self._set_direction('reverse' if self._direction == 'forward' else 'forward')
            self.get_logger().info(f'Next leg in {self._pause_sec} seconds: direction={self._direction} legs_remaining={self._legs_remaining}')
            self._schedule_next()
            return

        if self._loop:
            self.get_logger().info(f'Loop mode: restarting in {self._pause_sec} seconds.')
            self._schedule_next()

    def _schedule_next(self):
        if self._loop_timer is not None:
            self._loop_timer.cancel()
        self._loop_timer = self.create_timer(self._pause_sec, self._loop_once)

    def _loop_once(self):
        if self._loop_timer:
            self._loop_timer.cancel()
        try:
            self._prepare_active_route()
        except Exception as exc:
            self.get_logger().warn(f"Cannot start next leg yet: {exc}", throttle_duration_sec=2.0)
            self._schedule_next()
            return
        self._send_goal()

    def _feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        if not feedback:
            return

        base = (
            f"remaining_poses={getattr(feedback, 'number_of_poses_remaining', -1)} "
            f"distance_remaining={getattr(feedback, 'distance_remaining', float('nan')):.2f} m "
            f"recoveries={getattr(feedback, 'number_of_recoveries', -1)}"
        )

        now = time.monotonic()
        cmd = self._last_cmd_vel
        cmd_age = now - self._last_cmd_vel_time if self._last_cmd_vel_time else float('inf')
        cmd_nav = self._last_cmd_vel_nav
        cmd_nav_age = now - self._last_cmd_vel_nav_time if self._last_cmd_vel_nav_time else float('inf')

        extra = ""
        if cmd is not None:
            extra += f" cmd_vel=({cmd.linear.x:.2f},{cmd.angular.z:.2f}) age={cmd_age:.1f}s"
        if cmd_nav is not None:
            extra += f" cmd_vel_nav=({cmd_nav.linear.x:.2f},{cmd_nav.angular.z:.2f}) age={cmd_nav_age:.1f}s"

        self.get_logger().info(base + extra, throttle_duration_sec=1.0)

    def stop(self):
        self._shutdown = True
        if self._loop_timer:
            self._loop_timer.cancel()
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)


def default_route_path() -> str:
    return os.path.join(
        get_package_share_directory('amr_tools'),
        'config',
        'room_route.yaml',
    )


def _lifecycle_state_label(state_id: Optional[int]) -> str:
    return _LIFECYCLE_STATE_LABEL.get(state_id, str(state_id))


def _get_lifecycle_state_sync(node: Node, node_name: str, timeout_sec: float) -> Optional[int]:
    from lifecycle_msgs.srv import GetState

    name = node_name if node_name.startswith('/') else f'/{node_name}'
    client = node.create_client(GetState, f'{name}/get_state')
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return None

    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        return None
    response = future.result()
    if response is None:
        return None
    return int(response.current_state.id)


def _run_preflight(timeout_sec: float) -> bool:
    node = Node('route_loop_runner_preflight')
    try:
        required_nodes = ['/controller_server', '/planner_server', '/bt_navigator', '/velocity_smoother']
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            states: dict[str, Optional[int]] = {}
            all_active = True
            for name in required_nodes:
                state = _get_lifecycle_state_sync(node, name, timeout_sec=0.8)
                states[name] = state
                if state != 3:
                    all_active = False

            if all_active:
                node.get_logger().info('Preflight: all required Nav2 nodes are active.')
                return True

            for name in required_nodes:
                node.get_logger().info(f'Preflight: {name} state={_lifecycle_state_label(states[name])}')
            time.sleep(0.5)

        for name in required_nodes:
            state = _get_lifecycle_state_sync(node, name, timeout_sec=1.0)
            node.get_logger().error(f'Preflight failed: {name} state={_lifecycle_state_label(state)} (expected active)')
        node.get_logger().error(
            'Preflight failed. Set AMCL initial pose and wait. If lifecycle_manager aborted earlier, restart Nav2.'
        )
        return False
    finally:
        node.destroy_node()


def parse_args(argv: Optional[List[str]] = None):
    parser = argparse.ArgumentParser(description='Send a looped NavigateThroughPoses route.')
    parser.add_argument('--route', type=str, default=None, help='Route YAML file to load.')
    parser.add_argument('--frame-id', type=str, default='map', help='TF frame for poses.')
    parser.add_argument('--base-frame', type=str, default='base_link', help='Robot base frame for TF lookup.')
    parser.add_argument('--loop', action='store_true', help='Enable continuous looping.')
    parser.add_argument('--pause', type=float, default=3.0, help='Seconds to wait between loops.')
    parser.add_argument(
        '--yaw-mode',
        type=str,
        default='from_file',
        choices=['from_file', 'heading', 'zero'],
        help='How to set waypoint yaw: from_file|heading|zero (heading is often best for AGV routes).',
    )
    parser.add_argument(
        '--reverse',
        action='store_true',
        help='Reverse the waypoint order to drive back along the same route (use with --start-mode nearest).',
    )
    parser.add_argument(
        '--roundtrips',
        type=int,
        default=0,
        help='Run N roundtrips (forward then reverse) and stop. Not compatible with --loop.',
    )
    parser.add_argument(
        '--stop-on-failure',
        action='store_true',
        default=True,
        help='Stop after ABORTED/CANCELED instead of continuing to the next leg.',
    )
    parser.add_argument(
        '--no-stop-on-failure',
        action='store_false',
        dest='stop_on_failure',
        help='Continue to next leg even if a leg fails.',
    )
    parser.add_argument(
        '--start-mode',
        type=str,
        default='nearest',
        choices=['nearest', 'fixed'],
        help='Route start selection: nearest (recommended) or fixed index.',
    )
    parser.add_argument('--start-index', type=int, default=0, help='Start waypoint index if start-mode=fixed.')
    parser.add_argument('--start-tf-timeout', type=float, default=2.0, help='Seconds to wait for TF when start-mode=nearest.')
    parser.add_argument(
        '--no-preflight',
        action='store_true',
        help='Disable preflight lifecycle checks (useful for debugging).',
    )
    parser.add_argument(
        '--preflight-timeout',
        type=float,
        default=8.0,
        help='Seconds to wait for Nav2 lifecycle nodes to become active.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None):
    args = parse_args(argv)

    try:
        route_file = args.route or default_route_path()
        route_points, frame_id = load_route(route_file, args.frame_id)
    except Exception as exc:
        print(f"Failed to load route: {exc}", file=sys.stderr)
        return 1

    rclpy.init(args=argv)
    if not args.no_preflight:
        if not _run_preflight(timeout_sec=max(float(args.preflight_timeout), 1.0)):
            rclpy.shutdown()
            return 2

    if args.roundtrips > 0 and args.loop:
        print('Error: use either --loop or --roundtrips (not both).', file=sys.stderr)
        rclpy.shutdown()
        return 2

    node = RouteLoopRunner(
        route_points,
        frame_id,
        args.loop,
        args.pause,
        args.yaw_mode,
        args.base_frame,
        args.start_mode,
        args.start_index,
        args.start_tf_timeout,
        args.reverse,
        args.roundtrips,
        args.stop_on_failure,
    )

    def _handle_shutdown(signum, frame):
        node.get_logger().info('Shutdown requested, canceling goal...')
        node.stop()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_shutdown)
    signal.signal(signal.SIGTERM, _handle_shutdown)

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
