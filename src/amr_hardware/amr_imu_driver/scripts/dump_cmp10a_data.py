#!/usr/bin/env python3
"""Stream raw CMP10A IMU frames from the serial device and display them in hex."""

import argparse
import sys
from textwrap import wrap

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    print("pyserial is required to run this script:", exc, file=sys.stderr)
    sys.exit(1)


def format_hex(data: bytes) -> str:
    hex_pairs = wrap(data.hex(), 2)
    return ' '.join(hex_pairs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/imu', help='Serial device path (default: /dev/imu)')
    parser.add_argument('--baudrate', type=int, default=921600, help='Serial baud rate (default: 921600)')
    parser.add_argument('--timeout', type=float, default=0.1, help='Read timeout in seconds (default: 0.1)')
    parser.add_argument('--chunk', type=int, default=256, help='Number of bytes to read per iteration (default: 256)')
    parser.add_argument('--max-iterations', type=int, default=50, help='Stop after this many non-empty reads (<=0 for infinite).')
    args = parser.parse_args()

    try:
        with serial.Serial(port=args.port, baudrate=args.baudrate, timeout=args.timeout) as ser:
            print(f"Listening on {args.port} @ {args.baudrate} baud (chunk={args.chunk})")
            iterations = 0
            while args.max_iterations <= 0 or iterations < args.max_iterations:
                data = ser.read(args.chunk)
                if not data:
                    continue
                iterations += 1
                print(f"[{iterations}] {len(data)} bytes: {format_hex(data)}")
    except serial.SerialException as exc:
        print(f"Failed to read from {args.port}: {exc}", file=sys.stderr)
        sys.exit(2)


if __name__ == '__main__':
    main()
