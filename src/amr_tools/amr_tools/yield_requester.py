import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
import time
from typing import Optional
import wave

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def _finite_ranges_in_sector(scan: LaserScan, sector_min_rad: float, sector_max_rad: float):
    if not scan.ranges:
        return []
    a0 = float(scan.angle_min)
    inc = float(scan.angle_increment) if scan.angle_increment != 0.0 else 0.0
    if inc == 0.0:
        return []
    i0 = int(max(0, math.floor((sector_min_rad - a0) / inc)))
    i1 = int(min(len(scan.ranges) - 1, math.ceil((sector_max_rad - a0) / inc)))
    out = []
    rmin = float(scan.range_min)
    rmax = float(scan.range_max)
    for i in range(i0, i1 + 1):
        r = float(scan.ranges[i])
        if math.isfinite(r) and rmin < r < rmax:
            out.append(r)
    return out


class YieldRequester(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('yield_requester')
        self._min_distance_m = float(args.min_distance_m)
        self._front_angle_rad = math.radians(float(args.front_angle_deg))
        self._hold_sec = float(args.hold_sec)
        self._cooldown_sec = float(args.cooldown_sec)
        self._sound_mode = str(args.sound_mode)
        self._message = str(args.message)

        self._blocked_since: Optional[float] = None
        self._last_played = 0.0
        self._beep_path = self._ensure_beep_file()
        self._aplay = shutil.which('aplay')
        self._espeak = shutil.which('espeak')

        self.create_subscription(LaserScan, args.scan_topic, self._scan_cb, 10)

        self.get_logger().info(
            f"YieldRequester: scan_topic={args.scan_topic} min_distance_m={self._min_distance_m:.2f} "
            f"front_angle_deg={args.front_angle_deg:.1f} hold_sec={self._hold_sec:.1f} cooldown_sec={self._cooldown_sec:.1f} "
            f"sound_mode={self._sound_mode}"
        )

    def _ensure_beep_file(self) -> Optional[str]:
        """Create a small WAV beep in /tmp so we don't need any package assets."""
        try:
            path = f"/tmp/agv_yield_beep_{os.getpid()}.wav"
            sample_rate = 8000
            duration_sec = 0.20
            freq_hz = 880.0
            amplitude = 0.35
            n = int(sample_rate * duration_sec)
            with wave.open(path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(sample_rate)
                frames = bytearray()
                for i in range(n):
                    t = i / sample_rate
                    s = math.sin(2.0 * math.pi * freq_hz * t)
                    v = int(max(-1.0, min(1.0, s * amplitude)) * 32767)
                    frames += struct.pack('<h', v)
                wf.writeframes(bytes(frames))
            return path
        except Exception:
            return None

    def _play_sound(self):
        now = time.monotonic()
        if now - self._last_played < self._cooldown_sec:
            return
        self._last_played = now

        if self._sound_mode == 'none':
            self.get_logger().warn('Obstacle blocked: please clear the way.')
            return

        if self._sound_mode == 'espeak' and self._espeak:
            try:
                subprocess.run([self._espeak, self._message], check=False, timeout=3.0)
                return
            except Exception:
                pass

        if self._aplay and self._beep_path:
            try:
                subprocess.run([self._aplay, '-q', self._beep_path], check=False, timeout=3.0)
                return
            except Exception:
                pass

        # Fallback: terminal bell + log
        sys.stdout.write('\a')
        sys.stdout.flush()
        self.get_logger().warn('Obstacle blocked: please clear the way (no audio backend).')

    def _scan_cb(self, msg: LaserScan):
        sector = _finite_ranges_in_sector(msg, -self._front_angle_rad, self._front_angle_rad)
        if not sector:
            self._blocked_since = None
            return

        closest = min(sector)
        now = time.monotonic()
        if closest <= self._min_distance_m:
            if self._blocked_since is None:
                self._blocked_since = now
            if (now - self._blocked_since) >= self._hold_sec:
                self.get_logger().warn(
                    f"Blocked: obstacle at {closest:.2f} m (<= {self._min_distance_m:.2f} m). Requesting yield...",
                    throttle_duration_sec=1.0,
                )
                self._play_sound()
        else:
            self._blocked_since = None


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='Play a sound when an obstacle blocks the robot within a threshold.')
    p.add_argument('--scan-topic', type=str, default='/scan')
    p.add_argument('--min-distance-m', type=float, default=0.60, help='Trigger if obstacle is closer than this.')
    p.add_argument('--front-angle-deg', type=float, default=25.0, help='Front sector half-angle.')
    p.add_argument('--hold-sec', type=float, default=1.5, help='Obstacle must persist for this long before triggering.')
    p.add_argument('--cooldown-sec', type=float, default=6.0, help='Minimum seconds between sounds.')
    p.add_argument('--sound-mode', choices=['beep', 'espeak', 'none'], default='beep')
    p.add_argument('--message', type=str, default='Excuse me, please clear the way')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=None)
    node = YieldRequester(args)
    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
