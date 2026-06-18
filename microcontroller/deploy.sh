#!/usr/bin/env bash
set -e

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"

cd "$(dirname "$0")"

idf.py build
idf.py -p "$PORT" -b "$BAUD" flash monitor
