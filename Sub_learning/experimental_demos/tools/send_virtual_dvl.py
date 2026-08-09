#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Send learning-only virtual DVL frames to ArduSub over a serial URL."""

import argparse
import time


def xor_checksum(payload):
    """Return the byte-wise XOR used by the learning protocol."""
    result = 0
    for byte in payload.encode('ascii'):
        result ^= byte
    return result


def encode_payload(payload):
    """Frame an ASCII payload with checksum and line ending."""
    return f'${payload}*{xor_checksum(payload):02X}\r\n'.encode('ascii')


def encode_dvl(sequence, velocity_x, velocity_y, velocity_z, quality):
    """Encode one body-frame virtual DVL velocity sample."""
    payload = (
        f'DVLD,{sequence},{velocity_x:.3f},{velocity_y:.3f},'
        f'{velocity_z:.3f},{quality}'
    )
    return encode_payload(payload)


def parse_ack(frame):
    """Decode a valid Pixhawk status reply, or return None."""
    text = frame.decode('ascii').strip()
    if not text.startswith('$') or '*' not in text:
        return None

    payload, checksum_text = text[1:].rsplit('*', 1)
    if (
        len(checksum_text) != 2
        or xor_checksum(payload) != int(checksum_text, 16)
    ):
        return None

    fields = payload.split(',')
    if len(fields) != 5 or fields[0] != 'DVLA':
        return None

    return {
        'sequence': int(fields[1]),
        'healthy': bool(int(fields[2])),
        'good_frames': int(fields[3]),
        'bad_frames': int(fields[4]),
    }


def run_self_test():
    """Exercise protocol encoding and reply decoding without a serial port."""
    frame = encode_dvl(7, 0.25, -0.10, 0.05, 80)
    assert frame.startswith(b'$DVLD,7,0.250,-0.100,0.050,80*')

    ack = encode_payload('DVLA,7,1,8,2')
    decoded = parse_ack(ack)
    assert decoded == {
        'sequence': 7,
        'healthy': True,
        'good_frames': 8,
        'bad_frames': 2,
    }
    print('virtual DVL protocol self-test passed')


def parse_args():
    """Parse command-line options for hardware and SITL transports."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--port',
        help='COM8, /dev/ttyUSB0, or socket://127.0.0.1:6795',
    )
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument(
        '--rate', type=float, default=10.0, help='frames per second'
    )
    parser.add_argument(
        '--vx', type=float, default=0.20, help='body-forward velocity in m/s'
    )
    parser.add_argument(
        '--vy', type=float, default=0.00, help='body-right velocity in m/s'
    )
    parser.add_argument(
        '--vz', type=float, default=0.00, help='body-down velocity in m/s'
    )
    parser.add_argument(
        '--quality',
        type=int,
        default=80,
        help='0-100; zero reports unhealthy',
    )
    parser.add_argument(
        '--count', type=int, default=0, help='zero sends until Ctrl-C'
    )
    parser.add_argument('--self-test', action='store_true')
    return parser.parse_args()


def main():
    """Send samples until the count is reached or the user interrupts."""
    args = parse_args()
    if args.self_test:
        run_self_test()
        return
    if args.port is None:
        raise SystemExit('--port is required unless --self-test is used')
    if args.rate <= 0:
        raise SystemExit('--rate must be positive')
    if not 0 <= args.quality <= 100:
        raise SystemExit('--quality must be in the range 0-100')

    try:
        import serial
    except ImportError as error:
        raise SystemExit(
            'pyserial is required: python -m pip install pyserial'
        ) from error

    period_s = 1.0 / args.rate
    sequence = 0
    sent = 0

    with serial.serial_for_url(
        args.port, baudrate=args.baud, timeout=0.05
    ) as port:
        try:
            while args.count == 0 or sent < args.count:
                started = time.monotonic()
                frame = encode_dvl(
                    sequence, args.vx, args.vy, args.vz, args.quality
                )
                port.write(frame)
                print('TX', frame.decode('ascii').strip())

                ack = port.readline()
                if ack:
                    decoded = parse_ack(ack)
                    reply = (
                        decoded
                        if decoded is not None
                        else ack.decode('ascii', errors='replace').strip()
                    )
                    print('RX', reply)

                sequence = (sequence + 1) & 0xFFFFFFFF
                sent += 1
                time.sleep(max(0.0, period_s - (time.monotonic() - started)))
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
