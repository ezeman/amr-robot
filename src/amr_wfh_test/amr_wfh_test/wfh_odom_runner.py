import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass(frozen=True)
class _Leg:
    name: str
    target_distance_m: float
    linear_x: float


class WfhOdomRunner(Node):
    """Publish cmd_vel forward until odom distance is reached."""

    def __init__(self) -> None:
        super().__init__('wfh_odom_runner')

        self.declare_parameter('target_distance_m', 1.0)
        self.declare_parameter('linear_x', 0.20)
        self.declare_parameter('include_backward', False)
        self.declare_parameter('rounds', 1)
        self.declare_parameter('forward_distance_m', 1.0)
        self.declare_parameter('backward_distance_m', 1.0)
        self.declare_parameter('backward_linear_x', -0.20)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_duration_sec', 15.0)
        self.declare_parameter('max_total_duration_sec', 300.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('log_interval_sec', 0.2)
        self.declare_parameter('settle_time_sec', 0.5)

        self._target = float(self.get_parameter('target_distance_m').value)
        self._linear_x = float(self.get_parameter('linear_x').value)
        self._include_backward = bool(self.get_parameter('include_backward').value)
        self._rounds = int(self.get_parameter('rounds').value)
        self._forward_distance = float(self.get_parameter('forward_distance_m').value)
        self._backward_distance = float(self.get_parameter('backward_distance_m').value)
        self._backward_linear_x = float(self.get_parameter('backward_linear_x').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._max_duration = float(self.get_parameter('max_duration_sec').value)
        self._max_total_duration = float(self.get_parameter('max_total_duration_sec').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._log_interval = float(self.get_parameter('log_interval_sec').value)
        self._settle_time = float(self.get_parameter('settle_time_sec').value)

        if self._rounds < 1:
            self.get_logger().warn(f"Invalid rounds={self._rounds}, forcing to 1.")
            self._rounds = 1
        if self._include_backward:
            self._linear_x = abs(self._linear_x)
            if self._backward_linear_x >= 0.0:
                self._backward_linear_x = -abs(self._linear_x)
        else:
            self._linear_x = float(self._linear_x)

        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        self._start_pose: Optional[Tuple[float, float]] = None
        self._odom_frame: Optional[str] = None
        self._base_frame: Optional[str] = None
        self._latest_pose: Optional[Tuple[float, float]] = None
        self._start_time: Optional[float] = None
        self._leg_start_time: Optional[float] = None
        self._done = False
        self._zero_until: Optional[float] = None
        self._next_log_time = 0.0

        self._legs: List[_Leg] = self._build_legs()
        self._leg_index = 0

        period = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.05
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"WFH odom runner starting: rate={self._rate_hz:.1f} Hz, "
            f"odom='{self._odom_topic}', cmd_vel='{self._cmd_vel_topic}'"
        )
        if self._include_backward:
            self.get_logger().info(
                f"Mode: forward+backward, rounds={self._rounds}, "
                f"forward={self._forward_distance:.2f}m @ +{abs(self._linear_x):.2f}m/s, "
                f"backward={self._backward_distance:.2f}m @ {self._backward_linear_x:.2f}m/s"
            )
        else:
            self.get_logger().info(
                f"Mode: forward-only, target={self._target:.2f} m @ {self._linear_x:.2f} m/s"
            )

    def _build_legs(self) -> List[_Leg]:
        if not self._include_backward:
            return [_Leg(name='forward', target_distance_m=self._target, linear_x=self._linear_x)]

        legs: List[_Leg] = []
        for round_index in range(self._rounds):
            legs.append(_Leg(
                name=f'round{round_index + 1}_forward',
                target_distance_m=self._forward_distance,
                linear_x=abs(self._linear_x),
            ))
            legs.append(_Leg(
                name=f'round{round_index + 1}_backward',
                target_distance_m=self._backward_distance,
                linear_x=self._backward_linear_x,
            ))
        return legs

    def _on_odom(self, msg: Odometry) -> None:
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if not math.isfinite(px) or not math.isfinite(py):
            self.get_logger().warn('Ignoring odom with non-finite position.')
            return

        self._latest_pose = (px, py)
        if self._start_pose is None:
            self._start_pose = (px, py)
            self._odom_frame = msg.header.frame_id or ''
            self._base_frame = msg.child_frame_id or ''
            if self._odom_frame != 'odom':
                self.get_logger().warn(
                    f"Expected odom frame 'odom', got '{self._odom_frame}'. Using as-is."
                )
            if self._base_frame != 'base_link':
                self.get_logger().warn(
                    f"Expected base frame 'base_link', got '{self._base_frame}'. Using as-is."
                )
            self._start_time = time.monotonic()
            self._next_log_time = self._start_time
            self._start_pose = (px, py)
            self._leg_start_time = self._start_time
            self.get_logger().info(f"Captured start pose ({px:.3f}, {py:.3f}) in frame '{self._odom_frame}'")
            self.get_logger().info(f"Starting leg 1/{len(self._legs)}: {self._legs[0].name}")

    def _publish_cmd(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)

    def _on_timer(self) -> None:
        now = time.monotonic()

        if self._done:
            if self._zero_until is not None and now < self._zero_until:
                self._publish_cmd(0.0)
                return
            rclpy.shutdown()
            return

        # Settling between legs (publish zero cmd_vel for a short duration).
        if self._zero_until is not None and now < self._zero_until:
            self._publish_cmd(0.0)
            return
        if self._zero_until is not None and now >= self._zero_until:
            self._zero_until = None

        if self._start_pose is None or self._latest_pose is None:
            if now >= self._next_log_time:
                self.get_logger().info('Waiting for odom...')
                self._next_log_time = now + self._log_interval
            return

        if self._start_time is None:
            self._start_time = now
        if self._leg_start_time is None:
            self._leg_start_time = now

        elapsed_total = now - self._start_time
        elapsed_leg = now - self._leg_start_time
        dx = self._latest_pose[0] - self._start_pose[0]
        dy = self._latest_pose[1] - self._start_pose[1]
        dist = math.hypot(dx, dy)

        if now >= self._next_log_time:
            current_leg = self._legs[self._leg_index]
            self.get_logger().info(
                f"[{current_leg.name}] Progress: {dist:.3f} / {current_leg.target_distance_m:.3f} m "
                f"(leg {elapsed_leg:.1f}s, total {elapsed_total:.1f}s)"
            )
            self._next_log_time = now + self._log_interval

        current_leg = self._legs[self._leg_index]
        if dist >= current_leg.target_distance_m:
            self.get_logger().info(
                f"[{current_leg.name}] Reached target distance {dist:.3f} m "
                f"in {elapsed_leg:.2f} s. Stopping to settle."
            )
            self._advance_leg_or_finish()
            return

        if elapsed_leg >= self._max_duration:
            self.get_logger().warn(
                f"[{current_leg.name}] Timeout {elapsed_leg:.2f}s before reaching target "
                f"(dist={dist:.3f} m). Stopping."
            )
            self._finish(success=False)
            return
        if elapsed_total >= self._max_total_duration:
            self.get_logger().warn(
                f"Total timeout {elapsed_total:.2f}s exceeded (max_total_duration_sec). Stopping."
            )
            self._finish(success=False)
            return

        self._publish_cmd(current_leg.linear_x)

    def _finish(self, *, success: bool) -> None:
        self._done = True
        self._zero_until = time.monotonic() + self._settle_time
        self._publish_cmd(0.0)
        status = "SUCCESS" if success else "TIMEOUT"
        self.get_logger().info(
            f"WFH test result: {status}. Publishing zero cmd_vel for {self._settle_time:.2f}s."
        )

    def _advance_leg_or_finish(self) -> None:
        now = time.monotonic()
        self._publish_cmd(0.0)
        self._zero_until = now + self._settle_time

        self._leg_index += 1
        if self._leg_index >= len(self._legs):
            self.get_logger().info('All legs completed.')
            self._finish(success=True)
            return

        # Start next leg from the current pose after settling.
        if self._latest_pose is not None:
            self._start_pose = self._latest_pose
        self._leg_start_time = now
        self.get_logger().info(f"Next leg {self._leg_index + 1}/{len(self._legs)}: {self._legs[self._leg_index].name}")
        # Let the settle phase publish zero cmd_vel before the next command is sent.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WfhOdomRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WFH odom runner interrupted. Stopping.')
        node._publish_cmd(0.0)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
