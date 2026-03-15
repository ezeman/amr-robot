import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
import math
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from rcl_interfaces.msg import SetParametersResult

# === Defaults (overridable via ROS params) ===
# Measured wheel ground-contact radius (diameter 0.20 m) and track width (0.40 m).
DEFAULT_WHEEL_RADIUS = 0.10   # meters
DEFAULT_BASE_WIDTH = 0.40     # meters
# Scaling factors tuned from field test (1 m forward -> 1.0 m, 90° turn -> 90°).
DEFAULT_ODOM_LINEAR_SCALE = 0.10
DEFAULT_ODOM_ANGULAR_SCALE = 1.02
PORT = "/dev/rs485port"
BAUD = 115200
UNIT_ID = 3

# === Registers ===
REG_MODE = 0x200D
REG_CTRL = 0x200E
REG_TGT_VEL_L = 0x2088
REG_TGT_VEL_R = 0x2089
REG_ACT_VEL_L = 0x20AB
REG_ACT_VEL_R = 0x20AC

# === Conversion ===
def s16_to_u16(val: int) -> int:
    return val & 0xFFFF

def u16_to_s16(val: int) -> int:
    return val - 0x10000 if val & 0x8000 else val

# === Modbus-safe wrappers ===
def write_reg(client, addr, val, unit_id):
    last = None
    for kw in ({"unit": unit_id}, {"slave": unit_id}, {}):
        try:
            return client.write_register(address=addr, value=val, **kw)
        except TypeError as e: last = e
        except Exception as e: last = e; break
    raise last if last else RuntimeError("write_reg failed")

def read_holding(client, addr, count, unit_id):
    last = None
    for kw in ({"unit": unit_id}, {"slave": unit_id}, {}):
        try:
            return client.read_holding_registers(address=addr, count=count, **kw)
        except TypeError as e: last = e
        except Exception as e: last = e; break
    raise last if last else RuntimeError("read_holding failed")

def stop_drive(client, unit_id):
    try:
        write_reg(client, REG_CTRL, 0x07, unit_id)
        write_reg(client, REG_CTRL, 0x00, unit_id)
    except Exception:
        pass

# === Main Node ===
class ZLAC8015DNode(Node):
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            v = value.strip().lower()
            if v in ('true', '1', 'yes', 'y', 'on'):
                return True
            if v in ('false', '0', 'no', 'n', 'off'):
                return False
        return bool(value)

    def __init__(self):
        super().__init__('zlac8015d')
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.wheel_radius = self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS).value
        self.base_width = self.declare_parameter('base_width', DEFAULT_BASE_WIDTH).value
        self.odom_linear_scale = self.declare_parameter('odom_linear_scale', DEFAULT_ODOM_LINEAR_SCALE).value
        self.odom_angular_scale = self.declare_parameter('odom_angular_scale', DEFAULT_ODOM_ANGULAR_SCALE).value
        # Sign / wiring compatibility knobs (keep defaults for existing robots).
        # - invert_right_motor: keep the current right-wheel inversion behavior.
        # - angular_sign: set to -1.0 if robot rotates opposite of ROS convention (CCW is +).
        self.invert_right_motor = self._as_bool(self.declare_parameter('invert_right_motor', True).value)
        self.angular_sign = float(self.declare_parameter('angular_sign', 1.0).value)
        self.get_logger().info(
            f"Base params: angular_sign={self.angular_sign:.3f} invert_right_motor={self.invert_right_motor} "
            f"base_width={self.base_width:.3f} wheel_radius={self.wheel_radius:.3f}"
        )
        # Allow tuning at runtime without restart.
        self.add_on_set_parameters_callback(self._on_params_set)
        self.unit_id = UNIT_ID
        self.client = ModbusSerialClient(
            port=PORT,
            baudrate=BAUD,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=0.5
        )
        if not self.client.connect():
            self.get_logger().error("❌ RS485 connection failed")
            raise RuntimeError("Modbus connection failed")
        self.get_logger().info("✅ RS485 connected")

        # Init velocity mode
        write_reg(self.client, REG_MODE, 3, self.unit_id)
        write_reg(self.client, REG_CTRL, 0x08, self.unit_id)

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.br = TransformBroadcaster(self) if self.publish_tf else None
        self.timer = self.create_timer(0.05, self.odom_callback)  # 20Hz

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.ros_clock = Clock(clock_type=ClockType.ROS_TIME)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_time_ros = self.ros_clock.now()
        self.last_time_steady = self.steady_clock.now()

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z * self.angular_sign
        vL = v - (w * self.base_width / 2)
        vR = v + (w * self.base_width / 2)
        rpmL = int((vL / (2 * math.pi * self.wheel_radius)) * 60)
        rpmR = int((vR / (2 * math.pi * self.wheel_radius)) * 60)

        if self.invert_right_motor:
            rpmR = -rpmR

        write_reg(self.client, REG_TGT_VEL_L, s16_to_u16(rpmL), self.unit_id)
        write_reg(self.client, REG_TGT_VEL_R, s16_to_u16(rpmR), self.unit_id)

    def odom_callback(self):
        try:
            rr = read_holding(self.client, REG_ACT_VEL_L, 2, self.unit_id)
            if not hasattr(rr, "registers") or len(rr.registers) < 2:
                return
            rpmL = u16_to_s16(rr.registers[0])
            rpmR = u16_to_s16(rr.registers[1])

            if self.invert_right_motor:
                rpmR = -rpmR

            vL = (rpmL / 60) * 2 * math.pi * self.wheel_radius
            vR = (rpmR / 60) * 2 * math.pi * self.wheel_radius

            # Apply optional scale factors to compensate for over/under-estimated odom.
            vL *= self.odom_linear_scale
            vR *= self.odom_linear_scale
            vx = (vL + vR) / 2.0
            vth = (vR - vL) / self.base_width
            vth *= self.odom_angular_scale
            vth *= self.angular_sign

            now_ros = self.ros_clock.now()
            now_steady = self.steady_clock.now()
            dt = (now_steady - self.last_time_steady).nanoseconds / 1e9
            if dt <= 0.0:
                self.get_logger().warn(
                    f"Skipping odom integration due to non-positive dt ({dt:.6f}s). Check system clock synchronization.",
                    throttle_duration_sec=5.0
                )
                self.last_time_steady = now_steady
                self.last_time_ros = now_ros
                return
            self.last_time_steady = now_steady
            self.last_time_ros = now_ros

            delta_x = vx * math.cos(self.th) * dt
            delta_y = vx * math.sin(self.th) * dt
            delta_th = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # Odometry msg
            odom = Odometry()
            odom.header.stamp = now_ros.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0

            q = tf_transformations.quaternion_from_euler(0, 0, self.th)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            self.odom_pub.publish(odom)

            # TF
            if self.br is not None:
                t = TransformStamped()
                t.header.stamp = now_ros.to_msg()
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.br.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"❗ Odometry read error: {e}")

    # Live-parameter update so tuning doesn't require restart.
    def _on_params_set(self, params):
        for p in params:
            if p.name == 'odom_linear_scale':
                self.odom_linear_scale = float(p.value)
            elif p.name == 'odom_angular_scale':
                self.odom_angular_scale = float(p.value)
            elif p.name == 'wheel_radius':
                self.wheel_radius = float(p.value)
            elif p.name == 'base_width':
                self.base_width = float(p.value)
            elif p.name == 'invert_right_motor':
                self.invert_right_motor = self._as_bool(p.value)
            elif p.name == 'angular_sign':
                self.angular_sign = float(p.value)
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    try:
        node = ZLAC8015DNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"❌ Fatal error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
