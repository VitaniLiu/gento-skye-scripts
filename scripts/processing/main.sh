#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$ROOT/tests/tmp"
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1
export HF_HOME="$ROOT/tests/cache/huggingface"
export HF_DATASETS_CACHE="$ROOT/tests/cache/huggingface/datasets"
export XDG_CACHE_HOME="$ROOT/tests/cache"
mkdir -p "$TMPDIR" "$HF_DATASETS_CACHE"
export PYTHONPATH="$ROOT/src"
exec "$ROOT/.venv/bin/python" "$ROOT/src/processing/main.py" "$@"
