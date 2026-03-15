import math
import re
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover
    serial = None
    SerialException = Exception


def _covariance_from_diagonal(diagonal: Sequence[float]) -> List[float]:
    """Expand a 3-element diagonal vector into a 3x3 row-major covariance matrix."""
    values = list(diagonal)
    if len(values) != 3:
        raise ValueError("Covariance diagonal must have exactly 3 elements")
    return [
        float(values[0]), 0.0, 0.0,
        0.0, float(values[1]), 0.0,
        0.0, 0.0, float(values[2]),
    ]


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
    """Convert roll, pitch, yaw (radians) to quaternion [x, y, z, w]."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


class SerialImuNode(Node):
    """Simple serial IMU driver that converts ASCII data into sensor_msgs/Imu."""

    def __init__(self) -> None:
        super().__init__('serial_imu_node')

        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic_name', 'imu/data_raw')
        self.declare_parameter('poll_interval', 0.01)
        self.declare_parameter('reconnect_delay', 2.0)
        self.declare_parameter('euler_in_degrees', True)
        self.declare_parameter(
            'angular_velocity_covariance_diagonal', [0.01, 0.01, 0.01])
        self.declare_parameter(
            'linear_acceleration_covariance_diagonal', [0.1, 0.1, 0.1])
        self.declare_parameter(
            'orientation_covariance_diagonal', [0.05, 0.05, 0.05])

        if serial is None:
            self.get_logger().error(
                "python3-serial is not available. Please install the 'python3-serial' package.")

        self._port: str = str(self.get_parameter('port').value)
        self._baudrate: int = int(float(self.get_parameter('baudrate').value))
        self._timeout: float = float(self.get_parameter('timeout').value)
        self._frame_id: str = str(self.get_parameter('frame_id').value)
        self._topic_name: str = str(self.get_parameter('topic_name').value)
        self._poll_interval: float = float(self.get_parameter('poll_interval').value)
        self._reconnect_delay: float = float(self.get_parameter('reconnect_delay').value)
        self._euler_in_degrees: bool = bool(self.get_parameter('euler_in_degrees').value)

        ang_cov_diag = list(self.get_parameter('angular_velocity_covariance_diagonal').value)
        lin_cov_diag = list(self.get_parameter('linear_acceleration_covariance_diagonal').value)
        ori_cov_diag = list(self.get_parameter('orientation_covariance_diagonal').value)

        try:
            self._angular_velocity_cov = _covariance_from_diagonal(ang_cov_diag)
            self._linear_acceleration_cov = _covariance_from_diagonal(lin_cov_diag)
            self._orientation_cov = _covariance_from_diagonal(ori_cov_diag)
        except ValueError as exc:
            self.get_logger().error(f'Invalid covariance configuration: {exc}')
            self._angular_velocity_cov = _covariance_from_diagonal([0.01, 0.01, 0.01])
            self._linear_acceleration_cov = _covariance_from_diagonal([0.1, 0.1, 0.1])
            self._orientation_cov = _covariance_from_diagonal([0.05, 0.05, 0.05])

        self._serial: Optional['serial.Serial'] = None
        self._next_reconnect_time: float = 0.0

        self._publisher = self.create_publisher(Imu, self._topic_name, 10)
        self._timer = self.create_timer(self._poll_interval, self._poll_serial)

        self._connect_serial(initial=True)

    def destroy_node(self) -> bool:
        """Close the serial port before shutting down."""
        if self._serial is not None:
            try:
                self._serial.close()
            except SerialException:
                pass
        return super().destroy_node()

    def _connect_serial(self, *, initial: bool = False) -> None:
        """Attempt to open the serial port."""
        if serial is None:
            return

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            self.get_logger().info(
                f"Connected to IMU on {self._port} @ {self._baudrate} baud.")
        except SerialException as exc:
            level = self.get_logger().error if initial else self.get_logger().warn
            level(f"Failed to open {self._port}: {exc}")
            self._serial = None
            self._next_reconnect_time = time.monotonic() + self._reconnect_delay

    def _maybe_reconnect(self) -> None:
        if self._serial is not None:
            return
        if time.monotonic() < self._next_reconnect_time:
            return
        self._connect_serial()

    def _poll_serial(self) -> None:
        """Timer callback that reads the next line from the serial port."""
        if self._serial is None or not self._serial.is_open:
            self._maybe_reconnect()
            return

        try:
            raw = self._serial.readline()
        except SerialException as exc:
            self.get_logger().warn(f"Serial read error: {exc}", throttle_duration_sec=2.0)
            try:
                self._serial.close()
            except SerialException:
                pass
            self._serial = None
            self._next_reconnect_time = time.monotonic() + self._reconnect_delay
            return

        if not raw:
            return

        try:
            line = raw.decode('utf-8', errors='ignore').strip()
        except UnicodeDecodeError:
            self.get_logger().warn('Received undecodable serial data, skipping line.')
            return

        if not line:
            return

        imu_msg = self._parse_line(line)
        if imu_msg is None:
            return

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self._frame_id

        self._publisher.publish(imu_msg)

    def _parse_line(self, line: str) -> Optional[Imu]:
        """Convert an ASCII line into an Imu message."""
        fields = re.split(r'[\s,;]+', line.strip())
        values: List[float] = []
        for field in fields:
            if not field:
                continue
            try:
                values.append(float(field))
            except ValueError:
                self.get_logger().debug(f"Non-numeric token '{field}' in line: '{line}'")
                return None

        if len(values) < 6:
            self.get_logger().warn(
                f"Expected at least 6 numeric values (accel+gyro), received {len(values)}: {line}",
                throttle_duration_sec=2.0,
            )
            return None

        msg = Imu()
        msg.linear_acceleration.x = values[0]
        msg.linear_acceleration.y = values[1]
        msg.linear_acceleration.z = values[2]
        msg.linear_acceleration_covariance = self._linear_acceleration_cov

        msg.angular_velocity.x = values[3]
        msg.angular_velocity.y = values[4]
        msg.angular_velocity.z = values[5]
        msg.angular_velocity_covariance = self._angular_velocity_cov

        orientation_set = False

        if len(values) >= 10:
            # Assume full quaternion (ax, ay, az, gx, gy, gz, qx, qy, qz, qw)
            msg.orientation.x = values[6]
            msg.orientation.y = values[7]
            msg.orientation.z = values[8]
            msg.orientation.w = values[9]
            orientation_set = True
        elif len(values) >= 9:
            # Assume Euler angles appended (ax..gz, roll, pitch, yaw)
            roll = values[6]
            pitch = values[7]
            yaw = values[8]
            if self._euler_in_degrees:
                roll = math.radians(roll)
                pitch = math.radians(pitch)
                yaw = math.radians(yaw)
            qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            orientation_set = True

        if orientation_set:
            msg.orientation_covariance = self._orientation_cov
        else:
            msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, -1.0,
            ]

        return msg


def main() -> None:
    rclpy.init()
    node = SerialImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU driver interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
