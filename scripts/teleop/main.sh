#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT/src"
exec "$ROOT/.venv/bin/python" "$ROOT/src/teleop/main.py" "$@"
