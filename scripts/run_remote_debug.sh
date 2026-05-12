#!/usr/bin/env sh
set -eu

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <target-ip> <port>" >&2
    exit 1
fi

TARGET_IP=$1
TARGET_PORT=$2

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR="$ROOT_DIR/build"
BINARY="$BUILD_DIR/ok3506_demo"

if [ ! -x "$BINARY" ]; then
    echo "Build the project first: scripts/build.sh" >&2
    exit 1
fi

HOST_GDB=${HOST_GDB:-gdb-multiarch}
if command -v "$HOST_GDB" >/dev/null 2>&1; then
    GDB="$HOST_GDB"
elif command -v gdb >/dev/null 2>&1; then
    GDB=gdb
else
    echo "Need gdb-multiarch or gdb on the host" >&2
    exit 1
fi

if ! "$GDB" -q -ex "set architecture arm" -ex quit >/dev/null 2>&1; then
    echo "Host debugger $GDB does not support ARM. Install gdb-multiarch." >&2
    exit 1
fi

exec "$GDB" "$BINARY" \
  -ex "set architecture arm" \
  -ex "target remote ${TARGET_IP}:${TARGET_PORT}"
