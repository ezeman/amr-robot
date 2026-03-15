import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def _yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class InitialPosePublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('set_initial_pose')
        self._args = args
        # AMCL in Nav2 typically subscribes to /initialpose with TRANSIENT_LOCAL durability.
        # Use TRANSIENT_LOCAL here so publishing works even if AMCL starts after this node.
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(PoseWithCovarianceStamped, args.topic, qos)

    def publish_once(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._args.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self._args.x)
        msg.pose.pose.position.y = float(self._args.y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _yaw_to_quat(float(self._args.yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = float(self._args.cov_xx)
        cov[7] = float(self._args.cov_yy)
        cov[35] = float(self._args.cov_yawyaw)
        msg.pose.covariance = cov

        self._pub.publish(msg)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish a single /initialpose (PoseWithCovarianceStamped) for AMCL.'
    )
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw in radians (map frame).')
    parser.add_argument('--frame', type=str, default='map')
    parser.add_argument('--topic', type=str, default='/initialpose')
    parser.add_argument('--repeat', type=int, default=5, help='Publish N times for robustness.')
    parser.add_argument('--rate_hz', type=float, default=10.0)
    parser.add_argument(
        '--wait_for_sub_sec',
        type=float,
        default=2.0,
        help='Wait up to N seconds for an /initialpose subscriber (AMCL). Transient-local publish still works without it.',
    )
    parser.add_argument('--cov_xx', type=float, default=0.25)
    parser.add_argument('--cov_yy', type=float, default=0.25)
    parser.add_argument('--cov_yawyaw', type=float, default=0.068)

    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = InitialPosePublisher(args)
    try:
        if args.repeat < 1:
            node.get_logger().warn('--repeat < 1, nothing to publish.')
            return 0

        deadline = time.monotonic() + max(0.0, float(args.wait_for_sub_sec))
        while time.monotonic() < deadline and rclpy.ok():
            if node._pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        period = 1.0 / max(args.rate_hz, 1e-3)
        node.get_logger().info(
            f"Publishing initial pose to {args.topic}: "
            f"x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f} rad, "
            f"repeat={args.repeat}, rate={args.rate_hz:.1f} Hz, "
            f"durability=TRANSIENT_LOCAL"
        )

        for _ in range(args.repeat):
            node.publish_once()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

        # Give DDS a moment to flush last sample.
        rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info('Done.')
        return 0
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
        return 130
    except Exception as exc:
        node.get_logger().error(f'Failed: {exc}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
