## 📁 Folder: amr_ws/src/zlac8015d_ros2_driver/zlac8015d_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from pymodbus.client.serial import ModbusSerialClient
from tf_transformations import quaternion_from_euler
import math

class ZLAC8015DNode(Node):
    def __init__(self):
        super().__init__('zlac8015d_node')
        self.client = ModbusSerialClient(
            method='rtu',
            port='/dev/ttyUSB0',
            baudrate=115200,
            stopbits=1,
            bytesize=8,
            parity='N',
            timeout=0.05
        )
        self.driver_id = 1
        self.base_width = 0.45
        self.wheel_radius = 0.1
        self.max_rpm = 1000
        self.cmd_timeout = 1.0
        self.last_cmd_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        if not self.client.connect():
            self.get_logger().error("❌ Failed to connect to ZLAC8015D driver.")
            exit(1)

        self.init_velocity_mode()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.timer = self.create_timer(0.05, self.update_odometry)

    def write_register(self, address, value):
        try:
            value &= 0xFFFF
            self.client.write_register(address, value, unit=self.driver_id)
        except Exception as e:
            self.get_logger().error(f"Write error at {hex(address)}: {e}")

    def read_register(self, address):
        try:
            result = self.client.read_holding_registers(address, 1, unit=self.driver_id)
            return result.registers[0] if result and not result.isError() else None
        except Exception as e:
            self.get_logger().warn(f"Read failed at {hex(address)}: {e}")
            return None

    def init_velocity_mode(self):
        self.write_register(0x200D, 3)
        self.write_register(0x2080, 300)
        self.write_register(0x2081, 300)
        self.write_register(0x2082, 300)
        self.write_register(0x2083, 300)
        self.write_register(0x200E, 8)
        self.get_logger().info("✅ Velocity mode enabled")

    def emergency_stop(self):
        self.write_register(0x200E, 5)
        self.get_logger().warn("🛑 Emergency stop!")

    def stop(self):
        self.write_register(0x2088, 0)
        self.write_register(0x2089, 0)
        self.write_register(0x200E, 7)

    def clear_fault(self):
        self.write_register(0x200E, 6)

    def set_velocity(self, left_rpm, right_rpm):
        left_rpm = max(min(left_rpm, self.max_rpm), -self.max_rpm)
        right_rpm = max(min(right_rpm, self.max_rpm), -self.max_rpm)
        self.write_register(0x2088, left_rpm)
        self.write_register(0x2089, right_rpm)

    def get_actual_velocity(self):
        l = self.read_register(0x20AB)
        r = self.read_register(0x20AC)
        def to_signed(val): return val - 0x10000 if val >= 0x8000 else val
        return to_signed(l or 0) / 10.0, to_signed(r or 0) / 10.0  # unit: RPM

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        l_rpm, r_rpm = self.get_actual_velocity()
        v_l = (l_rpm * 2 * math.pi * self.wheel_radius) / 60.0
        v_r = (r_rpm * 2 * math.pi * self.wheel_radius) / 60.0

        vx = (v_r + v_l) / 2.0
        vth = (v_r - v_l) / self.base_width

        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = Quaternion()
        q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        linear = msg.linear.x
        angular = msg.angular.z
        v_left = linear - (angular * self.base_width / 2)
        v_right = linear + (angular * self.base_width / 2)
        rpm_factor = 60 / (2 * math.pi * self.wheel_radius)
        left_rpm = int(v_left * rpm_factor)
        right_rpm = int(v_right * rpm_factor)
        self.set_velocity(left_rpm, right_rpm)


def main(args=None):
    rclpy.init(args=args)
    node = ZLAC8015DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.client.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
