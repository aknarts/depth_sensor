#!/usr/bin/env sh
set -eu

if ! command -v idf.py >/dev/null 2>&1; then
    echo "idf.py not found. Source ESP-IDF export.sh before running this script." >&2
    exit 127
fi

BUILD_DIR="${BUILD_DIR:-build}"

idf.py -B "$BUILD_DIR" build
