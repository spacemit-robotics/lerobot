#!/usr/bin/env bash
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CHASSIS_DIR="$ROOT_DIR/third_party/chassis"
BUILD_DIR="$CHASSIS_DIR/build"

if [[ ! -d "$CHASSIS_DIR" ]]; then
    echo "third_party/chassis not found: $CHASSIS_DIR" >&2
    exit 1
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake ..
cmake --build . -j"$(nproc)"

echo "Built chassis library at $BUILD_DIR/libchassis.so"