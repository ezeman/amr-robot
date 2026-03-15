#!/usr/bin/env python3
"""
Encoder Distance Calibration Test
==================================
Drives the AGV straight forward a fixed distance, then compares
the odometry (encoder) distance vs the commanded distance.

Usage:
  1. Place robot at a known starting mark on the floor.
  2. Run:  python3 test_encoder_distance.py [--distance 1.0] [--speed 0.15]
  3. Measure the actual distance the robot traveled with a tape measure.
  4. Compare the 3 values:
       - Commanded distance  (what was requested)
       - Encoder distance    (what /odom reported)
       - Measured distance   (what you measured)
  5. Correction factor = measured / encoder

Requires: agv_mapping or hardware.launch.py running.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class EncoderTestNode(Node):
    def __init__(self, target_dist: float, speed: float, turn_test: bool, turn_angle: float):
        super().__init__('encoder_test')
        self.target_dist = target_dist
        self.speed = speed
        self.turn_test = turn_test
        self.turn_angle = turn_angle  # radians

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.odom_count = 0
        self.done = False

    def _quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _odom_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self.odom_count += 1

        if self.start_x is None:
            self.start_x = self.cur_x
            self.start_y = self.cur_y
            self.start_yaw = self.cur_yaw

    def wait_for_odom(self, timeout=10.0):
        """Wait until first odom message arrives."""
        self.get_logger().info('Waiting for /odom ...')
        start = time.monotonic()
        while self.odom_count == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                self.get_logger().error(f'/odom not received within {timeout}s — is base_controller running?')
                return False
        self.get_logger().info(f'/odom OK  (x={self.cur_x:.4f}, y={self.cur_y:.4f}, yaw={math.degrees(self.cur_yaw):.1f}°)')
        return True

    def _send_cmd(self, lx, az):
        msg = Twist()
        msg.linear.x = lx
        msg.angular.z = az
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self._send_cmd(0.0, 0.0)
            time.sleep(0.05)

    def _traveled_dist(self):
        dx = self.cur_x - self.start_x
        dy = self.cur_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def _turned_angle(self):
        diff = self.cur_yaw - self.start_yaw
        # Normalize to [-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def run_linear_test(self):
        self.get_logger().info(f'=== LINEAR TEST: target={self.target_dist:.3f} m, speed={self.speed:.3f} m/s ===')
        self.get_logger().info('Starting in 2 seconds...')
        time.sleep(2.0)

        # Reset start position
        self.start_x = self.cur_x
        self.start_y = self.cur_y
        self.start_yaw = self.cur_yaw

        self.get_logger().info(f'Start pos: x={self.start_x:.4f}, y={self.start_y:.4f}')
        self.get_logger().info('Driving forward...')

        t0 = time.monotonic()
        max_time = (self.target_dist / self.speed) * 3  # safety timeout

        while not self.done:
            rclpy.spin_once(self, timeout_sec=0.02)
            dist = self._traveled_dist()

            if dist >= self.target_dist:
                self.done = True
                break

            if time.monotonic() - t0 > max_time:
                self.get_logger().warn(f'Timeout ({max_time:.1f}s) reached!')
                self.done = True
                break

            self._send_cmd(self.speed, 0.0)

        self.stop()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)

        encoder_dist = self._traveled_dist()
        elapsed = time.monotonic() - t0

        print('\n' + '=' * 55)
        print('  ENCODER DISTANCE CALIBRATION RESULT')
        print('=' * 55)
        print(f'  Commanded distance : {self.target_dist:.4f} m')
        print(f'  Encoder distance   : {encoder_dist:.4f} m')
        print(f'  Speed (commanded)  : {self.speed:.3f} m/s')
        print(f'  Actual avg speed   : {encoder_dist / elapsed:.3f} m/s')
        print(f'  Elapsed time       : {elapsed:.2f} s')
        print(f'  End pos: x={self.cur_x:.4f}, y={self.cur_y:.4f}')
        print('-' * 55)
        print('  >> Measure the ACTUAL distance with a tape measure <<')
        print('  >> Correction = actual_measured / encoder_distance  <<')
        print(f'  >> Then set odom_linear_scale = current * correction <<')
        print('=' * 55 + '\n')

    def run_turn_test(self):
        angle_deg = math.degrees(self.turn_angle)
        self.get_logger().info(f'=== TURN TEST: target={angle_deg:.1f}°, angular speed={self.speed:.3f} rad/s ===')
        self.get_logger().info('Starting in 2 seconds...')
        time.sleep(2.0)

        # Reset start position
        self.start_x = self.cur_x
        self.start_y = self.cur_y
        self.start_yaw = self.cur_yaw

        self.get_logger().info(f'Start yaw: {math.degrees(self.start_yaw):.1f}°')
        self.get_logger().info('Turning...')

        t0 = time.monotonic()
        ang_speed = self.speed if self.turn_angle > 0 else -self.speed
        max_time = (abs(self.turn_angle) / abs(self.speed)) * 3

        while not self.done:
            rclpy.spin_once(self, timeout_sec=0.02)
            turned = abs(self._turned_angle())

            if turned >= abs(self.turn_angle):
                self.done = True
                break

            if time.monotonic() - t0 > max_time:
                self.get_logger().warn(f'Timeout ({max_time:.1f}s) reached!')
                self.done = True
                break

            self._send_cmd(0.0, ang_speed)

        self.stop()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)

        encoder_angle = self._turned_angle()
        elapsed = time.monotonic() - t0

        print('\n' + '=' * 55)
        print('  ENCODER TURN CALIBRATION RESULT')
        print('=' * 55)
        print(f'  Commanded angle    : {angle_deg:.1f}°')
        print(f'  Encoder angle      : {math.degrees(encoder_angle):.1f}°')
        print(f'  Elapsed time       : {elapsed:.2f} s')
        print(f'  End yaw            : {math.degrees(self.cur_yaw):.1f}°')
        print('-' * 55)
        print('  >> Measure the ACTUAL angle (e.g. with floor marks)  <<')
        print('  >> Correction = actual_angle / encoder_angle         <<')
        print(f'  >> Then set odom_angular_scale = current * correction <<')
        print('=' * 55 + '\n')


def main():
    parser = argparse.ArgumentParser(description='Encoder distance/turn calibration test')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='Target linear distance in meters (default: 1.0)')
    parser.add_argument('--speed', type=float, default=0.15,
                        help='Linear speed m/s or angular speed rad/s (default: 0.15)')
    parser.add_argument('--turn', action='store_true',
                        help='Run turn test instead of linear test')
    parser.add_argument('--angle', type=float, default=90.0,
                        help='Target turn angle in degrees (default: 90)')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = EncoderTestNode(
            target_dist=args.distance,
            speed=args.speed,
            turn_test=args.turn,
            turn_angle=math.radians(args.angle),
        )

        if not node.wait_for_odom():
            return

        if args.turn:
            node.run_turn_test()
        else:
            node.run_linear_test()

    except KeyboardInterrupt:
        print('\nInterrupted — stopping robot...')
        if 'node' in dir():
            node.stop()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
