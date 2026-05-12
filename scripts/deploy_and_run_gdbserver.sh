#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR="$ROOT_DIR/build"
BINARY="$BUILD_DIR/ok3506_demo"

TARGET_HOST=${TARGET_HOST:-192.168.0.232}
TARGET_USER=${TARGET_USER:-root}
TARGET_PORT=${TARGET_PORT:-1234}
TARGET_PATH=${TARGET_PATH:-/root/ok3506_demo}
SSH_PORT=${SSH_PORT:-22}
SSH_OPTS=${SSH_OPTS:--o BatchMode=yes -o StrictHostKeyChecking=accept-new}

if [ ! -x "$BINARY" ]; then
    echo "Build the project first: scripts/build.sh" >&2
    exit 1
fi

if ! command -v scp >/dev/null 2>&1; then
    echo "scp is required on the host" >&2
    exit 1
fi

if ! command -v ssh >/dev/null 2>&1; then
    echo "ssh is required on the host" >&2
    exit 1
fi

printf 'Copying %s to %s@%s:%s\n' "$BINARY" "$TARGET_USER" "$TARGET_HOST" "$TARGET_PATH"
scp -P "$SSH_PORT" $SSH_OPTS "$BINARY" "$TARGET_USER@$TARGET_HOST:$TARGET_PATH"

printf 'Starting gdbserver on %s:%s\n' "$TARGET_HOST" "$TARGET_PORT"
printf 'Program output will appear in this VS Code task terminal.\n'
exec ssh -p "$SSH_PORT" $SSH_OPTS "$TARGET_USER@$TARGET_HOST" \
  "chmod +x '$TARGET_PATH' && exec gdbserver 0.0.0.0:$TARGET_PORT '$TARGET_PATH'"
