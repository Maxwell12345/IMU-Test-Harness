#!/usr/bin/env bash
set -e

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"
MODE="${3:-headless}"

cd "$(dirname "$0")"

# idf.py fullclean
idf.py build
# idf.py -p "$PORT" -b "$BAUD" erase-flash

if [ "$MODE" = "monitor" ]; then
    idf.py -p "$PORT" -b "$BAUD" flash monitor
else
    idf.py -p "$PORT" -b "$BAUD" flash
fi