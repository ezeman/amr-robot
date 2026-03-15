import argparse
import time

import rclpy
from rclpy.node import Node


class PreflightTopics(Node):
    def __init__(self, topics: list[str], timeout_sec: float, check_period_sec: float) -> None:
        super().__init__('preflight_topics')
        self._topics = topics
        self._timeout_sec = timeout_sec
        self._check_period_sec = check_period_sec

    def run(self) -> int:
        deadline = time.monotonic() + self._timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            missing = [t for t in self._topics if len(self.get_publishers_info_by_topic(t)) == 0]
            if not missing:
                self.get_logger().info(f'Preflight OK. Publishers detected on topics: {self._topics}')
                return 0
            self.get_logger().info(
                f'Waiting publishers on topics: {missing}',
                throttle_duration_sec=1.0,
            )
            rclpy.spin_once(self, timeout_sec=self._check_period_sec)

        missing = [t for t in self._topics if len(self.get_publishers_info_by_topic(t)) == 0]
        self.get_logger().error(f'Preflight FAILED. Missing publishers on topics: {missing}')
        return 2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Preflight check for required ROS2 topic publishers.')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--odom-topic', default='/wheel/odom')
    parser.add_argument('--timeout-sec', type=float, default=8.0)
    parser.add_argument('--check-period-sec', type=float, default=0.3)
    args = parser.parse_args(argv)

    topics = [args.scan_topic, args.imu_topic, args.odom_topic]

    rclpy.init()
    node = PreflightTopics(topics=topics, timeout_sec=max(args.timeout_sec, 0.1), check_period_sec=max(args.check_period_sec, 0.05))
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
