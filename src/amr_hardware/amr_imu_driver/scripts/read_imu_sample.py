#!/usr/bin/env python3
"""Read a few lines from the IMU serial port for format inspection."""

import argparse
import sys
import time

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    print("pyserial is required to run this script:", exc, file=sys.stderr)
    sys.exit(1)


def read_samples(port: str, baudrate: int, timeout: float, count: int, max_wait: float) -> None:
    try:
        with serial.Serial(port=port, baudrate=baudrate, timeout=timeout) as ser:
            print(f"Opened {port} @ {baudrate} baud; reading {count} line(s) ...")
            lines_read = 0
            start = time.time()
            while lines_read < count:
                line = ser.readline()
                if not line:
                    if time.time() - start > timeout * 5:
                        print("No data received yet; still waiting ...")
                        start = time.time()
                    continue
                try:
                    decoded = line.decode('utf-8', errors='replace').rstrip('\r\n')
                except UnicodeDecodeError:
                    decoded = repr(line)
                print(f"[{lines_read + 1}] {decoded}")
                lines_read += 1
    except serial.SerialException as exc:
        print(f"Failed to read from {port}: {exc}", file=sys.stderr)
        sys.exit(2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/imu', help='Serial device path (default: /dev/imu)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Serial baud rate (default: 115200)')
    parser.add_argument('--timeout', type=float, default=0.5, help='Read timeout in seconds (default: 0.5)')
    parser.add_argument('--count', type=int, default=10, help='Number of lines to capture (default: 10)')
    parser.add_argument('--max-wait', type=float, default=15.0, help='Maximum seconds to wait for new data before giving up (<=0 disables).')
    args = parser.parse_args()

    read_samples(args.port, args.baudrate, args.timeout, args.count, args.max_wait)


if __name__ == '__main__':
    main()
