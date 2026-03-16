#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class Ina219Reader:
    REG_CONFIG = 0x00
    REG_BUS_VOLTAGE = 0x02

    def __init__(self, bus_id: int, address: int):
        # Prefer smbus2; fallback to smbus if available on platform.
        try:
            from smbus2 import SMBus  # type: ignore
        except ImportError:
            from smbus import SMBus  # type: ignore
        self._bus = SMBus(bus_id)
        self._address = address
        self._configure()

    def _configure(self):
        # 32V bus range, gain /8 (320mV), 12-bit ADC for bus+shunt, continuous mode.
        config = 0x399F
        self._bus.write_i2c_block_data(
            self._address,
            self.REG_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF],
        )

    def read_bus_voltage(self) -> float:
        data = self._bus.read_i2c_block_data(self._address, self.REG_BUS_VOLTAGE, 2)
        raw = (data[0] << 8) | data[1]
        # Bus voltage LSB is 4mV at bits [15:3]
        return ((raw >> 3) * 0.004)


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('publish_rate_hz', 2.0)
        # LiFePO4 8S profile defaults.
        self.declare_parameter('battery_voltage_min', 22.4)
        self.declare_parameter('battery_voltage_max', 29.2)
        # Calibrated from field reading: meter 27.2V vs raw 10.68V => scale ~2.547.
        self.declare_parameter('voltage_scale', 2.547)
        self.declare_parameter('topic_name', '/battery_state')
        self.declare_parameter('battery_design_capacity_ah', 10.0)
        self.declare_parameter('i2c_retry_sec', 5.0)
        self.declare_parameter('percentage_mode', 'lifepo4_8s')

        bus_id = int(self.get_parameter('i2c_bus').value)
        addr_value = self.get_parameter('i2c_address').value
        if isinstance(addr_value, str):
            self._address = int(addr_value, 0)
        else:
            self._address = int(addr_value)

        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._v_min = float(self.get_parameter('battery_voltage_min').value)
        self._v_max = float(self.get_parameter('battery_voltage_max').value)
        self._v_scale = float(self.get_parameter('voltage_scale').value)
        self._design_capacity_ah = float(self.get_parameter('battery_design_capacity_ah').value)
        self._retry_sec = float(self.get_parameter('i2c_retry_sec').value)
        self._percentage_mode = str(self.get_parameter('percentage_mode').value)
        self._bus_id = bus_id

        topic_name = str(self.get_parameter('topic_name').value)
        self._pub = self.create_publisher(BatteryState, topic_name, 10)

        self._reader: Optional[Ina219Reader] = None
        self._last_retry_sec = 0.0
        self._try_init_reader(log_error=True)

        period = 1.0 / max(self._rate_hz, 0.1)
        self._timer = self.create_timer(period, self._on_timer)

    def _on_timer(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._reader is None and (now_sec - self._last_retry_sec) >= max(self._retry_sec, 1.0):
            self._try_init_reader(log_error=False)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Defaults when sensor read is unavailable.
        msg.voltage = float('nan')
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = self._design_capacity_ah
        msg.percentage = float('nan')
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        if self._reader is None:
            self._pub.publish(msg)
            return

        try:
            voltage = self._reader.read_bus_voltage() * self._v_scale
            msg.voltage = float(voltage)
            msg.percentage = self._voltage_to_percentage(voltage)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        except Exception as exc:
            self.get_logger().warn(f'Battery read failed: {exc}', throttle_duration_sec=5.0)

        self._pub.publish(msg)

    def _try_init_reader(self, log_error: bool):
        self._last_retry_sec = self.get_clock().now().nanoseconds / 1e9
        try:
            self._reader = Ina219Reader(self._bus_id, self._address)
            self.get_logger().info(
                f'INA219 initialized on i2c-{self._bus_id} addr={hex(self._address)}'
            )
        except Exception as exc:
            self._reader = None
            if log_error:
                self.get_logger().error(f'Cannot initialize INA219: {exc}')
            else:
                self.get_logger().warn(
                    f'INA219 still unavailable: {exc}',
                    throttle_duration_sec=max(self._retry_sec, 1.0),
                )

    def _voltage_to_percentage(self, voltage: float) -> float:
        if not math.isfinite(voltage):
            return float('nan')
        if self._percentage_mode == 'lifepo4_8s':
            return self._lifepo4_8s_percentage(voltage)
        if self._v_max <= self._v_min:
            return float('nan')
        x = (voltage - self._v_min) / (self._v_max - self._v_min)
        return max(0.0, min(1.0, x))

    def _lifepo4_8s_percentage(self, voltage: float) -> float:
        # 8S LiFePO4 OCV-like curve (pack voltage V -> SOC 0..1).
        # This gives more realistic mid-range SOC than linear mapping.
        curve = [
            (22.4, 0.00),
            (24.0, 0.05),
            (25.6, 0.12),
            (26.0, 0.22),
            (26.2, 0.32),
            (26.3, 0.42),
            (26.4, 0.52),
            (26.5, 0.62),
            (26.6, 0.72),
            (26.8, 0.82),
            (27.0, 0.90),
            (27.6, 0.97),
            (28.4, 1.00),
        ]
        if voltage <= curve[0][0]:
            return 0.0
        if voltage >= curve[-1][0]:
            return 1.0
        for i in range(1, len(curve)):
            v0, p0 = curve[i - 1]
            v1, p1 = curve[i]
            if voltage <= v1:
                if v1 <= v0:
                    return p1
                t = (voltage - v0) / (v1 - v0)
                return max(0.0, min(1.0, p0 + (p1 - p0) * t))
        return 1.0


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
