import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def _angle_normalize(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _angle_diff(a: float, b: float) -> float:
    return _angle_normalize(a - b)


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float  # radians


class WaypointRecorder(Node):
    """Record waypoints from TF map->base_link with different spacing on straight vs turns."""

    def __init__(self) -> None:
        super().__init__('record_waypoints')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_path', '')
        self.declare_parameter('straight_interval_m', 1.0)
        self.declare_parameter('turn_interval_m', 0.3)
        self.declare_parameter('turning_curvature_threshold_rad_per_m', 0.35)
        self.declare_parameter('min_step_m', 0.02)
        self.declare_parameter('sample_rate_hz', 20.0)
        self.declare_parameter('max_duration_sec', 0.0)
        self.declare_parameter('max_points', 0)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._output_path = str(self.get_parameter('output_path').value)
        self._straight_interval = float(self.get_parameter('straight_interval_m').value)
        self._turn_interval = float(self.get_parameter('turn_interval_m').value)
        self._curvature_thresh = float(self.get_parameter('turning_curvature_threshold_rad_per_m').value)
        self._min_step = float(self.get_parameter('min_step_m').value)
        self._sample_rate = float(self.get_parameter('sample_rate_hz').value)
        self._max_duration = float(self.get_parameter('max_duration_sec').value)
        self._max_points = int(self.get_parameter('max_points').value)

        if self._straight_interval <= 0.0:
            raise ValueError('straight_interval_m must be > 0')
        if self._turn_interval <= 0.0:
            raise ValueError('turn_interval_m must be > 0')
        if self._turn_interval > self._straight_interval:
            self.get_logger().warn('turn_interval_m is greater than straight_interval_m; this is unusual.')
        if self._sample_rate <= 0.0:
            self._sample_rate = 20.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._start_time = time.monotonic()
        self._last_sample: Optional[Waypoint] = None
        self._last_record: Optional[Waypoint] = None
        self._recorded: List[Waypoint] = []
        self._distance_since_record = 0.0
        self._turning = False
        self._next_log_time = self._start_time

        period = 1.0 / self._sample_rate
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Recording TF {self._map_frame} -> {self._base_frame} "
            f"(straight={self._straight_interval:.2f}m, turn={self._turn_interval:.2f}m, "
            f"curvature>{self._curvature_thresh:.2f} rad/m). Ctrl+C to stop and save."
        )
        if not self._output_path:
            self.get_logger().warn("output_path is empty; will save to ./waypoints.yaml on shutdown.")

    def _lookup(self) -> Optional[Waypoint]:
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        if not all(math.isfinite(v) for v in (tx, ty, q.x, q.y, q.z, q.w)):
            return None
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        return Waypoint(x=float(tx), y=float(ty), yaw=float(yaw))

    def _should_stop(self) -> bool:
        if self._max_points > 0 and len(self._recorded) >= self._max_points:
            return True
        if self._max_duration > 0.0 and (time.monotonic() - self._start_time) >= self._max_duration:
            return True
        return False

    def _current_interval(self) -> float:
        return self._turn_interval if self._turning else self._straight_interval

    def _tick(self) -> None:
        now = time.monotonic()

        if self._should_stop():
            self.get_logger().info('Stop condition reached; saving waypoints.')
            self._save()
            rclpy.shutdown()
            return

        wp = self._lookup()
        if wp is None:
            if now >= self._next_log_time:
                self.get_logger().info('Waiting for TF map->base_link...', throttle_duration_sec=2.0)
                self._next_log_time = now + 0.5
            return

        if self._last_sample is None:
            self._last_sample = wp
            self._last_record = wp
            self._recorded.append(wp)
            self.get_logger().info(f"Recorded first waypoint at ({wp.x:.3f}, {wp.y:.3f}) yaw={math.degrees(wp.yaw):.1f}°")
            return

        dx = wp.x - self._last_sample.x
        dy = wp.y - self._last_sample.y
        ds = math.hypot(dx, dy)

        if ds < self._min_step:
            return

        dyaw = _angle_diff(wp.yaw, self._last_sample.yaw)
        curvature = abs(dyaw) / max(ds, 1e-6)
        self._turning = curvature >= self._curvature_thresh

        self._distance_since_record += ds
        self._last_sample = wp

        interval = self._current_interval()
        if self._distance_since_record >= interval:
            self._recorded.append(wp)
            self._distance_since_record = 0.0
            self._last_record = wp
            if now >= self._next_log_time:
                mode = 'TURN' if self._turning else 'STRAIGHT'
                self.get_logger().info(
                    f"[{mode}] waypoint #{len(self._recorded)} at ({wp.x:.3f}, {wp.y:.3f}) "
                    f"yaw={math.degrees(wp.yaw):.1f}°"
                )
                self._next_log_time = now + 0.2

    def _resolved_output_path(self) -> str:
        path = self._output_path.strip()
        if not path:
            path = os.path.join(os.getcwd(), 'waypoints.yaml')
        path = os.path.expanduser(path)
        return path

    def _save(self) -> None:
        out_path = self._resolved_output_path()
        out_dir = os.path.dirname(out_path) or '.'
        os.makedirs(out_dir, exist_ok=True)

        data = {
            'frame_id': self._map_frame,
            'waypoints': [
                {'x': round(wp.x, 4), 'y': round(wp.y, 4), 'yaw_deg': round(math.degrees(wp.yaw), 2)}
                for wp in self._recorded
            ],
        }

        tmp_path = out_path + '.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(tmp_path, out_path)
        self.get_logger().info(f"Saved {len(self._recorded)} waypoints to {out_path}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; saving waypoints.')
        try:
            node._save()
        except Exception as exc:
            node.get_logger().error(f'Failed to save waypoints: {exc}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

