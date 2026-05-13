#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if command -v python3 >/dev/null 2>&1; then
    exec python3 "$SCRIPT_DIR/check_all.py" "$@"
elif command -v python >/dev/null 2>&1; then
    exec python "$SCRIPT_DIR/check_all.py" "$@"
elif command -v py >/dev/null 2>&1; then
    exec py -3 "$SCRIPT_DIR/check_all.py" "$@"
else
    echo "python not found; install Python 3 or add it to PATH" >&2
    exit 127
fi
