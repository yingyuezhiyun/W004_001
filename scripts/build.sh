#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR="$ROOT_DIR/build"
TOOLCHAIN_FILE="$ROOT_DIR/cmake/ok3506-toolchain.cmake"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BUILD_DIR" -j"$(getconf _NPROCESSORS_ONLN)"
