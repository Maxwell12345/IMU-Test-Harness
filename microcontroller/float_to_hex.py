#!/usr/bin/env python3
################################################################################
# File:         float_to_hex.py
#
# Author:       Brian R. Atkinson
# Organization: Marine Corps Software Factory
# Created On:   06/22/26
# Description:  Converts a decimal floating point value into the little-endian
#               hexadecimal byte string used for C++ float or double storage.
#
################################################################################

import argparse
import struct


def parse_bool(value):
    normalized = value.strip().lower()

    if normalized in ("true", "t", "1", "yes", "y"):
        return True

    if normalized in ("false", "f", "0", "no", "n"):
        return False

    raise argparse.ArgumentTypeError("isDouble must be true or false")


def main():
    parser = argparse.ArgumentParser(
        description="Print the little-endian hexadecimal byte representation of a C++ float or double."
    )
    parser.add_argument("num", type=float, help="Decimal floating point value to encode.")
    parser.add_argument(
        "isDouble",
        nargs="?",
        type=parse_bool,
        default=False,
        help="When true, encode as an 8-byte C++ double. Defaults to false for a 4-byte C++ float.",
    )
    args = parser.parse_args()

    format_string = "<d" if args.isDouble else "<f"
    print(struct.pack(format_string, args.num).hex())


if __name__ == "__main__":
    main()
