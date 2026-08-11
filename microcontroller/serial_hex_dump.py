#!/usr/bin/env python3
################################################################################
# File:         serial_hex_dump.py
#
# Author:       Brian R. Atkinson
# Organization: Marine Corps Software Factory
# Created On:   07/21/26
# Description:  Reads raw bytes from a POSIX serial device and writes the received
#               data to stdout as space-separated hexadecimal byte values. When
#               requested, the same formatted hex output is also appended to a file.
#
################################################################################

import argparse
import os
import sys
import termios


BAUD_RATES = {
    50: termios.B50,
    75: termios.B75,
    110: termios.B110,
    134: termios.B134,
    150: termios.B150,
    200: termios.B200,
    300: termios.B300,
    600: termios.B600,
    1200: termios.B1200,
    1800: termios.B1800,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: getattr(termios, "B460800", None),
    500000: getattr(termios, "B500000", None),
    576000: getattr(termios, "B576000", None),
    921600: getattr(termios, "B921600", None),
    1000000: getattr(termios, "B1000000", None),
    1152000: getattr(termios, "B1152000", None),
    1500000: getattr(termios, "B1500000", None),
    2000000: getattr(termios, "B2000000", None),
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read bytes from a serial device and print them as hexadecimal bytes."
    )
    parser.add_argument(
        "port",
        nargs="?",
        default="/dev/ttyUSB0",
        help="Serial device path. Defaults to /dev/ttyUSB0.",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. Defaults to 115200.",
    )
    parser.add_argument(
        "-c",
        "--chunk-size",
        type=int,
        default=4096,
        help="Maximum bytes to read and print per stdout write. Defaults to 4096.",
    )
    parser.add_argument(
        "--no-flush",
        action="store_true",
        help="Do not flush stdout after each chunk. This improves throughput when piping output.",
    )
    parser.add_argument(
        "-f",
        "--file",
        action="store_true",
        help="Also write each formatted hex line to a file.",
    )
    parser.add_argument(
        "--name",
        type=str,
        default="/tmp/output.txt",
        help="Output file path used with -f/--file. Defaults to /tmp/output.txt.",
    )
    return parser.parse_args()


def get_baud_constant(baud):
    baud_constant = BAUD_RATES.get(baud)

    if baud_constant is None:
        supported = ", ".join(str(rate) for rate, value in BAUD_RATES.items() if value is not None)
        raise ValueError(f"unsupported baud rate {baud}; supported rates: {supported}")

    return baud_constant


def configure_serial_port(fd, baud):
    baud_constant = get_baud_constant(baud)
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_constant
    attrs[5] = baud_constant
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


def open_serial_port(port, baud):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY)

    try:
        configure_serial_port(fd, baud)
    except Exception:
        os.close(fd)
        raise

    return fd


def open_text_file(name):
    return open(name, "w", encoding="ascii")


def dump_hex_to_file(fd, hex_line):
    fd.write(hex_line)
    fd.write("\n")
    fd.flush()


def dump_hex_bytes(fd, chunk_size, flush_output, print_to_file, file_name):
    stdout = sys.stdout.buffer
    output_file = None

    try:
        if print_to_file:
            output_file = open_text_file(file_name)

        while True:
            data = os.read(fd, chunk_size)

            if not data:
                continue

            hex_line = data.hex(" ")
            stdout.write(hex_line.encode("ascii"))
            stdout.write(b"\n")

            if print_to_file:
                dump_hex_to_file(output_file, hex_line)

            if flush_output:
                stdout.flush()
    finally:
        if output_file is not None:
            output_file.close()


def main():
    args = parse_args()

    if args.chunk_size <= 0:
        print("chunk size must be greater than zero", file=sys.stderr)
        return 2

    fd = None

    try:
        fd = open_serial_port(args.port, args.baud)
        dump_hex_bytes(fd, args.chunk_size, not args.no_flush, args.file, args.name)
    except KeyboardInterrupt:
        return 0
    except OSError as error:
        print(f"serial port error: {error}", file=sys.stderr)
        return 1
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    finally:
        if fd is not None:
            os.close(fd)

    return 0


if __name__ == "__main__":
    sys.exit(main())
