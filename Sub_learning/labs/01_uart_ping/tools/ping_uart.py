#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""Send one PING line to the Pixhawk1 lab UART and require a PONG reply."""

import argparse
import sys

import serial


def parse_args() -> argparse.Namespace:
    """Parse command-line options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'port',
        help='Windows example: COM6; Linux example: /dev/ttyUSB0',
    )
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    """Run one request/reply transaction."""
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        port.reset_input_buffer()
        port.write(b'PING\n')
        port.flush()
        reply = port.readline().decode('ascii', errors='replace').strip()

    print(f'reply={reply!r}')
    if reply != 'PONG':
        message = (
            'Expected PONG. Check SERIAL2 settings, TX/RX crossing and '
            'the selected COM port.'
        )
        print(message, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
