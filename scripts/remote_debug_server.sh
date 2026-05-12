#!/usr/bin/env sh
set -eu

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 <binary-path> <port> [args...]" >&2
    exit 1
fi

BINARY_PATH=$1
PORT=$2
shift 2

if ! command -v gdbserver >/dev/null 2>&1; then
    echo "gdbserver is not installed on the target" >&2
    exit 1
fi

exec gdbserver 0.0.0.0:"$PORT" "$BINARY_PATH" "$@"
