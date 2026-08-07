#!/usr/bin/env bash
################################################################################
# File:         build.sh
#
# Author:       Brian R. Atkinson
# Organization: Marine Corps Software Factory
# Created On:   08/07/26
# Description:  Configures and builds either the production application or the
#               recorded-sensor simulation in a controlled host environment.
#
################################################################################
set -euo pipefail

BUILD_DIR="build"
SIMULATION_MODE="OFF"
CLEAN=0

if [[ "${1:-}" == "s" ]]; then
    SIMULATION_MODE="ON"
fi

for arg in "$@"; do
    case "$arg" in
        clean|--clean)
            CLEAN=1
            ;;
    esac
done

HOST_ENV=(
    env -i
    HOME="$HOME"
    USER="${USER:-$(id -un)}"
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    CC=/usr/bin/gcc
    CXX=/usr/bin/g++
    AS=/usr/bin/as
)

if [ "$CLEAN" -eq 1 ]; then
    rm -rf "$BUILD_DIR"
fi

"${HOST_ENV[@]}" cmake -S . -B "$BUILD_DIR" \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DSIMULATION_MODE="$SIMULATION_MODE"

cp test/WMM.COF "$BUILD_DIR/WMM.COF"

"${HOST_ENV[@]}" cmake --build "$BUILD_DIR" -j"$(nproc)"
