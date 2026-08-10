#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""Send CRC-protected six-axis lab frames and verify Pixhawk ACK packets."""

import argparse
import struct
import sys
import time
from collections.abc import Sequence

import serial


def crc16_ccitt(data: bytes) -> int:
    """Return CRC16-CCITT with initial value 0xFFFF."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(sequence: int, axes: Sequence[int], enabled: bool) -> bytes:
    """Encode one fixed-size command frame."""
    flags = 1 if enabled else 0
    payload = bytes([flags]) + struct.pack('<6h', *axes)
    body = bytes([1, 0x31, sequence, len(payload)]) + payload
    return b'\xAA\x55' + body + struct.pack('<H', crc16_ccitt(body))


def parse_args() -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'port',
        help='Windows example: COM6; Linux example: /dev/ttyUSB0',
    )
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument(
        '--rate', type=float, default=20.0, help='Frames per second'
    )
    parser.add_argument('--count', type=int, default=40)
    parser.add_argument(
        '--enable',
        action='store_true',
        help='Set the frame enable/deadman bit',
    )
    parser.add_argument(
        '--enable-after',
        type=float,
        help='Send disabled frames first, then enable after this many seconds',
    )
    parser.add_argument(
        '--axes',
        nargs=6,
        type=int,
        metavar=('ROLL', 'PITCH', 'YAW', 'HEAVE', 'FORWARD', 'LATERAL'),
        default=(0, 0, 0, 0, 0, 0),
        help='Six signed values in the closed interval [-1000, 1000]',
    )
    return parser.parse_args()


def main() -> int:
    """Send frames at a fixed rate and verify every ACK."""
    args = parse_args()
    if args.rate <= 0 or args.count <= 0:
        raise SystemExit('--rate and --count must be positive')
    if args.enable_after is not None and args.enable_after < 0:
        raise SystemExit('--enable-after must be non-negative')
    if any(value < -1000 or value > 1000 for value in args.axes):
        raise SystemExit('every axis must be between -1000 and 1000')

    period_s = 1.0 / args.rate
    missed = 0
    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        port.reset_input_buffer()
        start_time = time.monotonic()
        for index in range(args.count):
            sequence = index & 0xFF
            elapsed_s = time.monotonic() - start_time
            enable_after_elapsed = (
                args.enable_after is not None
                and elapsed_s >= args.enable_after
            )
            enabled = args.enable or enable_after_elapsed
            port.write(build_frame(sequence, args.axes, enabled))
            port.flush()
            ack = port.read(3)
            expected = bytes([0xAC, sequence, 0x00])
            if ack != expected:
                missed += 1
                ack_text = ack.hex() or '<timeout>'
                print(f'seq={sequence:3d} bad_ack={ack_text}')
            else:
                print(f'seq={sequence:3d} ack=ok')
            time.sleep(period_s)

    print(f'sent={args.count} missed={missed}')
    return 1 if missed else 0


if __name__ == '__main__':
    sys.exit(main())
