#!/usr/bin/env python3
"""ROS 2 driver for the Yahboom CMP10A 10-axis IMU."""

import math
import struct
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial  # type: ignore
    from serial import SerialException  # type: ignore
except ImportError:  # pragma: no cover
    serial = None
    SerialException = Exception  # type: ignore

G = 9.80665  # m/s^2

HEADER = 0x55
TYPE_TIME = 0x50
TYPE_ACC = 0x51
TYPE_GYRO = 0x52
TYPE_ANGLE = 0x53
TYPE_QUAT = 0x59
RRATE_MAP = {
    0.2: 0x01,
    0.5: 0x02,
    1.0: 0x03,
    2.0: 0x04,
    5.0: 0x05,
    10.0: 0x06,
    20.0: 0x07,
    50.0: 0x08,
    100.0: 0x09,
    200.0: 0x0B,
}


def _covariance_from_diagonal(values: List[float]) -> List[float]:
    diag = list(values)
    if len(diag) != 3:
        raise ValueError('Covariance diagonal must contain exactly 3 values')
    return [
        float(diag[0]), 0.0, 0.0,
        0.0, float(diag[1]), 0.0,
        0.0, 0.0, float(diag[2]),
    ]


def _read_short(lo: int, hi: int) -> int:
    unsigned = (hi << 8) | lo
    return struct.unpack('<h', struct.pack('<H', unsigned))[0]


class CMP10ADriver(Node):
    """Binary protocol driver for the Yahboom CMP10A IMU."""

    def __init__(self) -> None:
        super().__init__('cmp10a_driver')

        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic_name', 'imu/data')
        self.declare_parameter('configure_device', False)
        self.declare_parameter('output_rate_hz', 100.0)
        self.declare_parameter('use_quaternion_if_available', True)
        self.declare_parameter('linear_acceleration_covariance_diagonal', [0.02, 0.02, 0.04])
        self.declare_parameter('log_first_message', True)
        self.declare_parameter('angular_velocity_covariance_diagonal', [0.02, 0.02, 0.02])
        self.declare_parameter('orientation_covariance_diagonal', [0.02, 0.02, 0.05])

        self._port = str(self.get_parameter('port').value)
        self._baud = int(float(self.get_parameter('baud').value))
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._topic_name = str(self.get_parameter('topic_name').value)
        self._configure = bool(self.get_parameter('configure_device').value)
        self._rate_hz = float(self.get_parameter('output_rate_hz').value)
        self._use_quat = bool(self.get_parameter('use_quaternion_if_available').value)
        self._log_first_message = bool(self.get_parameter('log_first_message').value)

        try:
            lin_diag = list(self.get_parameter('linear_acceleration_covariance_diagonal').value)
            ang_diag = list(self.get_parameter('angular_velocity_covariance_diagonal').value)
            ori_diag = list(self.get_parameter('orientation_covariance_diagonal').value)
            self._lin_cov = _covariance_from_diagonal(lin_diag)
            self._ang_cov = _covariance_from_diagonal(ang_diag)
            self._ori_cov = _covariance_from_diagonal(ori_diag)
        except ValueError as exc:
            self.get_logger().warn(f'Invalid covariance configuration: {exc}; using defaults.')
            self._lin_cov = _covariance_from_diagonal([0.02, 0.02, 0.04])
            self._ang_cov = _covariance_from_diagonal([0.02, 0.02, 0.02])
            self._ori_cov = _covariance_from_diagonal([0.02, 0.02, 0.05])

        if serial is None:
            self.get_logger().error('python3-serial is not installed; CMP10A driver cannot start.')
            raise RuntimeError('pyserial missing')

        self._lock = threading.Lock()
        self._acc: List[float] = [0.0, 0.0, 0.0]
        self._gyro: List[float] = [0.0, 0.0, 0.0]
        self._euler: Optional[List[float]] = None
        self._quat: Optional[List[float]] = None
        self._acc_ready = False
        self._gyro_ready = False
        self._warn_no_data = False

        try:
            self.get_logger().info(f'Opening {self._port} @ {self._baud} baud')
            self._serial = serial.Serial(self._port, self._baud, timeout=0.1)
        except SerialException as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error(f'Failed to open {self._port}: {exc}')
            raise

        if self._configure:
            try:
                self._configure_sensor()
                self.get_logger().info('CMP10A configuration commands sent.')
            except SerialException as exc:
                self.get_logger().warn(f'Configuration attempt failed: {exc}')

        self._publisher = self.create_publisher(Imu, self._topic_name, 10)

        period = 1.0 / self._rate_hz if self._rate_hz > 0 else 0.01
        self._timer = self.create_timer(period, self._publish_imu)

        self._first_publish_logged = False
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    # --- Device configuration helpers -------------------------------------------------
    def _write_register(self, address: int, value: int) -> None:
        packet = bytes([
            0xFF,
            0xAA,
            address & 0xFF,
            value & 0xFF,
            (value >> 8) & 0xFF,
        ])
        self._serial.write(packet)
        self._serial.flush()
        time.sleep(0.01)

    def _configure_sensor(self) -> None:
        self._write_register(0x69, 0xB588)  # unlock
        rsw_val = 0
        rsw_val |= 1 << 0  # TIME
        rsw_val |= 1 << 1  # ACC
        rsw_val |= 1 << 2  # GYRO
        rsw_val |= 1 << 3  # ANGLE
        rsw_val |= 1 << 5  # PORT
        rsw_val |= 1 << 9  # QUATERNION
        self._write_register(0x02, rsw_val)
        rate_code = RRATE_MAP.get(self._rate_hz, 0x09)
        self._write_register(0x03, rate_code)

    # --- Serial read loop -------------------------------------------------------------
    def _read_exactly(self, size: int) -> bytes:
        data = bytearray()
        deadline = time.time() + 0.1
        while len(data) < size and not self._stop_event.is_set():
            chunk = self._serial.read(size - len(data))
            if chunk:
                data.extend(chunk)
            if time.time() > deadline:
                break
        return bytes(data)

    def _reader_loop(self) -> None:  # pragma: no cover - requires hardware
        while not self._stop_event.is_set():
            lead = self._serial.read(1)
            if not lead or lead[0] != HEADER:
                continue
            rest = self._read_exactly(10)
            if len(rest) != 10:
                continue
            frame_type = rest[0]
            payload = rest[1:9]
            checksum = rest[9]
            computed = (HEADER + sum(rest[:9])) & 0xFF
            if computed != checksum:
                continue
            words = [_read_short(payload[i], payload[i + 1]) for i in range(0, 8, 2)]
            with self._lock:
                if frame_type == TYPE_ACC:
                    ax = words[0] / 32768.0 * 16.0 * G
                    ay = words[1] / 32768.0 * 16.0 * G
                    az = words[2] / 32768.0 * 16.0 * G
                    self._acc = [ax, ay, az]
                    self._acc_ready = True
                elif frame_type == TYPE_GYRO:
                    gx = math.radians(words[0] / 32768.0 * 2000.0)
                    gy = math.radians(words[1] / 32768.0 * 2000.0)
                    gz = math.radians(words[2] / 32768.0 * 2000.0)
                    self._gyro = [gx, gy, gz]
                    self._gyro_ready = True
                elif frame_type == TYPE_ANGLE:
                    roll = math.radians(words[0] / 32768.0 * 180.0)
                    pitch = math.radians(words[1] / 32768.0 * 180.0)
                    yaw = math.radians(words[2] / 32768.0 * 180.0)
                    self._euler = [roll, pitch, yaw]
                elif frame_type == TYPE_QUAT:
                    q0 = words[0] / 32768.0
                    q1 = words[1] / 32768.0
                    q2 = words[2] / 32768.0
                    q3 = words[3] / 32768.0
                    self._quat = [q1, q2, q3, q0]

    # --- Publishing -------------------------------------------------------------------
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> List[float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

    def _publish_imu(self) -> None:
        with self._lock:
            acc_ready = self._acc_ready
            gyro_ready = self._gyro_ready
            acc = list(self._acc)
            gyro = list(self._gyro)
            quat = list(self._quat) if self._quat is not None else None
            euler = list(self._euler) if self._euler is not None else None
        if not (acc_ready and gyro_ready):
            if not self._warn_no_data:
                self.get_logger().warn('CMP10A has not produced accelerometer/gyro data yet; waiting...')
                self._warn_no_data = True
            return
        self._warn_no_data = False
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        if self._use_quat and quat is not None:
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat
            msg.orientation_covariance = self._ori_cov
        elif euler is not None:
            qx, qy, qz, qw = self._euler_to_quaternion(*euler)
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            msg.orientation_covariance = self._ori_cov
        else:
            msg.orientation_covariance[0] = -1.0
            msg.orientation_covariance[4] = -1.0
            msg.orientation_covariance[8] = -1.0
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.angular_velocity_covariance = self._ang_cov
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.linear_acceleration_covariance = self._lin_cov
        self._publisher.publish(msg)

        if self._log_first_message and not self._first_publish_logged:
            orientation_source = 'quat' if (self._use_quat and quat is not None) else ('euler' if euler is not None else 'unset')
            self.get_logger().info(
                'First IMU sample: acc=(%.3f, %.3f, %.3f) m/s^2, gyro=(%.3f, %.3f, %.3f) rad/s, orientation_source=%s' %
                (acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], orientation_source)
            )
            self._first_publish_logged = True

    # --- Shutdown ---------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if hasattr(self, '_serial') and self._serial:
            try:
                self._serial.close()
            except SerialException:  # pragma: no cover
                pass
        return super().destroy_node()


def main() -> None:  # pragma: no cover - entry point
    rclpy.init()
    node = CMP10ADriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
