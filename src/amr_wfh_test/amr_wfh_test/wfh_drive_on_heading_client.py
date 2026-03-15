import rclpy
from action_msgs.msg import GoalStatus
from typing import Optional
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import DriveOnHeading
from rclpy.action import ActionClient
from rclpy.node import Node


class WfhDriveOnHeadingClient(Node):
    """Send a DriveOnHeading goal to move forward a fixed distance."""

    def __init__(self) -> None:
        super().__init__('wfh_drive_on_heading_client')

        self.declare_parameter('distance_m', 1.0)
        self.declare_parameter('speed_mps', 0.20)
        self.declare_parameter('time_allowance_sec', 15.0)
        self.declare_parameter('action_name', 'drive_on_heading')

        distance = float(self.get_parameter('distance_m').value)
        speed = float(self.get_parameter('speed_mps').value)
        time_allowance = float(self.get_parameter('time_allowance_sec').value)
        action_name = str(self.get_parameter('action_name').value)

        self._goal_sent = False
        self._client = ActionClient(self, DriveOnHeading, action_name)

        self.get_logger().info(
            f"WFH DriveOnHeading client configured: dist={distance:.2f} m, "
            f"speed={speed:.2f} m/s, time_allowance={time_allowance:.1f}s, action='{action_name}'"
        )

        self._goal = DriveOnHeading.Goal()
        self._goal.target = Point(x=distance, y=0.0, z=0.0)
        self._goal.speed = speed
        self._goal.time_allowance = Duration(sec=int(time_allowance), nanosec=int((time_allowance % 1.0) * 1e9))

        self._connect_timer = self.create_timer(0.5, self._maybe_send_goal)

    def _maybe_send_goal(self) -> None:
        if self._goal_sent:
            return
        if not self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for drive_on_heading action server...', throttle_duration_sec=5.0)
            return

        self._goal_sent = True
        self._connect_timer.cancel()
        self.get_logger().info('Action server ready. Sending DriveOnHeading goal.')
        send_future = self._client.send_goal_async(self._goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('DriveOnHeading goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('DriveOnHeading goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error('Failed to get result from DriveOnHeading.')
            rclpy.shutdown()
            return

        elapsed = result.result.total_elapsed_time
        elapsed_sec = elapsed.sec + elapsed.nanosec / 1e9
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"DriveOnHeading succeeded in {elapsed_sec:.2f}s")
        else:
            self.get_logger().warn(f"DriveOnHeading finished with status {status} (elapsed {elapsed_sec:.2f}s)")
        rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WfhDriveOnHeadingClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WFH DriveOnHeading client interrupted.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
