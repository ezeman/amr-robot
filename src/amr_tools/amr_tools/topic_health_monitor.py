import time
from typing import Dict, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String


class TopicHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__('topic_health_monitor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('timeout_sec', 1.5)
        self.declare_parameter('check_period_sec', 0.5)
        self.declare_parameter('publish_topic', '/amr/health_ok')
        self.declare_parameter('detail_topic', '/amr/health_detail')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('fail_hard', False)
        self.declare_parameter('fail_grace_sec', 8.0)

        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._imu_topic = str(self.get_parameter('imu_topic').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._watch_topics: List[str] = [self._scan_topic, self._imu_topic, self._odom_topic]
        self._timeout_sec = max(float(self.get_parameter('timeout_sec').value), 0.1)
        self._check_period_sec = max(float(self.get_parameter('check_period_sec').value), 0.1)
        self._publish_topic = str(self.get_parameter('publish_topic').value)
        self._detail_topic = str(self.get_parameter('detail_topic').value)
        self._diagnostics_topic = str(self.get_parameter('diagnostics_topic').value)
        self._fail_hard = bool(self.get_parameter('fail_hard').value)
        self._fail_grace_sec = max(float(self.get_parameter('fail_grace_sec').value), 0.1)

        self._last_seen: Dict[str, float] = {topic: 0.0 for topic in self._watch_topics}
        self._unhealthy_since: float | None = None

        self._health_pub = self.create_publisher(Bool, self._publish_topic, 10)
        self._detail_pub = self.create_publisher(String, self._detail_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, self._diagnostics_topic, 10)

        self._scan_sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._scan_cb,
            10,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            self._imu_topic,
            self._imu_cb,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_cb,
            10,
        )

        self._timer = self.create_timer(self._check_period_sec, self._check_health)

        self.get_logger().info(
            f'Topic health monitor started. topics={self._watch_topics}, '
            f'timeout={self._timeout_sec:.2f}s, period={self._check_period_sec:.2f}s, '
            f'fail_hard={self._fail_hard}, fail_grace_sec={self._fail_grace_sec:.2f}s'
        )

    def _scan_cb(self, _msg: LaserScan) -> None:
        self._touch(self._scan_topic)

    def _imu_cb(self, _msg: Imu) -> None:
        self._touch(self._imu_topic)

    def _odom_cb(self, _msg: Odometry) -> None:
        self._touch(self._odom_topic)

    def _touch(self, topic: str) -> None:
        self._last_seen[topic] = time.monotonic()

    def _check_health(self) -> None:
        now = time.monotonic()
        stale_topics: List[str] = []

        for topic in self._watch_topics:
            last_seen = self._last_seen.get(topic, 0.0)
            age = now - last_seen if last_seen > 0.0 else float('inf')
            if age > self._timeout_sec:
                stale_topics.append(topic)

        health_ok = len(stale_topics) == 0
        self._health_pub.publish(Bool(data=health_ok))
        detail = 'healthy' if health_ok else f'stale_topics={stale_topics}'
        self._detail_pub.publish(String(data=detail))

        status = DiagnosticStatus()
        status.name = 'amr/topic_health'
        status.hardware_id = 'amr_topics'
        status.level = DiagnosticStatus.OK if health_ok else DiagnosticStatus.WARN
        status.message = 'All watched topics are healthy' if health_ok else 'Some watched topics are stale'

        values: List[KeyValue] = []
        for topic in self._watch_topics:
            last_seen = self._last_seen.get(topic, 0.0)
            age = now - last_seen if last_seen > 0.0 else float('inf')
            values.append(KeyValue(key=f'{topic}.age_sec', value='inf' if age == float('inf') else f'{age:.2f}'))
        values.append(KeyValue(key='stale_topics', value=','.join(stale_topics) if stale_topics else '-'))
        status.values = values

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [status]
        self._diag_pub.publish(diag)

        if not health_ok:
            self.get_logger().warn(
                f'Stale topics detected: {stale_topics}',
                throttle_duration_sec=2.0,
            )
            if self._unhealthy_since is None:
                self._unhealthy_since = now
            if self._fail_hard and (now - self._unhealthy_since) >= self._fail_grace_sec:
                raise RuntimeError(
                    f'Fail-hard triggered after {self._fail_grace_sec:.1f}s: stale topics {stale_topics}'
                )
        else:
            self._unhealthy_since = None


def main() -> int:
    rclpy.init()
    node = TopicHealthMonitor()
    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        node.get_logger().error(f'Topic health monitor fatal error: {exc}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
