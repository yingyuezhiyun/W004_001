#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR="$ROOT_DIR/build"
BINARY="$BUILD_DIR/ok3506_demo"

TARGET_HOST=${TARGET_HOST:-192.168.0.232}
TARGET_USER=${TARGET_USER:-root}
TARGET_PORT=${TARGET_PORT:-1234}
TARGET_PATH=${TARGET_PATH:-/root/ok3506_demo}
TARGET_ARGS=${TARGET_ARGS:-}
SSH_PORT=${SSH_PORT:-22}
SSH_OPTS=${SSH_OPTS:--o BatchMode=yes -o StrictHostKeyChecking=accept-new}
SSH_AUTO_UPDATE_KNOWN_HOSTS=${SSH_AUTO_UPDATE_KNOWN_HOSTS:-1}

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

cleanup_known_host() {
    if command -v ssh-keygen >/dev/null 2>&1; then
        ssh-keygen -R "[$TARGET_HOST]:$SSH_PORT" >/dev/null 2>&1 || true
        ssh-keygen -R "$TARGET_HOST" >/dev/null 2>&1 || true
    fi
}

if [ "$SSH_AUTO_UPDATE_KNOWN_HOSTS" = "1" ]; then
    cleanup_known_host
fi

printf 'Copying %s to %s@%s:%s\n' "$BINARY" "$TARGET_USER" "$TARGET_HOST" "$TARGET_PATH"
if ! scp -P "$SSH_PORT" $SSH_OPTS "$BINARY" "$TARGET_USER@$TARGET_HOST:$TARGET_PATH"; then
    if [ "$SSH_AUTO_UPDATE_KNOWN_HOSTS" = "1" ]; then
        echo "Retrying after removing stale SSH host key for $TARGET_HOST" >&2
        cleanup_known_host
        scp -P "$SSH_PORT" $SSH_OPTS "$BINARY" "$TARGET_USER@$TARGET_HOST:$TARGET_PATH"
    else
        exit 1
    fi
fi

printf 'Starting gdbserver on %s:%s\n' "$TARGET_HOST" "$TARGET_PORT"
printf 'Program output will appear in this VS Code task terminal.\n'
shell_quote() {
    printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\''/g")"
}

REMOTE_COMMAND="chmod +x $(shell_quote "$TARGET_PATH") && exec gdbserver 0.0.0.0:$TARGET_PORT $(shell_quote "$TARGET_PATH")"

TARGET_ARGS_TRIMMED=$(printf '%s' "$TARGET_ARGS" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')
if [ -n "$TARGET_ARGS_TRIMMED" ]; then
    # Split TARGET_ARGS on whitespace and append each token as its own argv entry.
    # Example: "-f config.conf" -> argv[1] == "-f" argv[2] == "config.conf"
    set -f
    set -- $TARGET_ARGS_TRIMMED
    set +f
    for target_arg do
        REMOTE_COMMAND="$REMOTE_COMMAND $(shell_quote "$target_arg")"
    done
fi

if [ "${DEBUG_DEPLOY:-0}" != "0" ]; then
    printf 'DEBUG: TARGET_ARGS=<%s>\n' "$TARGET_ARGS" >&2
    printf 'DEBUG: REMOTE_COMMAND=<%s>\n' "$REMOTE_COMMAND" >&2
fi

if ! ssh -p "$SSH_PORT" $SSH_OPTS "$TARGET_USER@$TARGET_HOST" "$REMOTE_COMMAND"; then
    if [ "$SSH_AUTO_UPDATE_KNOWN_HOSTS" = "1" ]; then
        echo "Retrying after removing stale SSH host key for $TARGET_HOST" >&2
        cleanup_known_host
        exec ssh -p "$SSH_PORT" $SSH_OPTS "$TARGET_USER@$TARGET_HOST" \
                    "$REMOTE_COMMAND"
    fi
    exit 1
fi
