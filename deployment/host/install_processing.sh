#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
export TMPDIR="$ROOT/tests/tmp" PIP_CACHE_DIR="$ROOT/tests/cache" PYTHONDONTWRITEBYTECODE=1
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR"
test -x "$ROOT/.venv/bin/python" || python3 -m venv "$ROOT/.venv"
"$ROOT/.venv/bin/python" -m pip install --timeout 120 --index-url https://download.pytorch.org/whl/cpu torch==2.7.1 torchvision==0.22.1
"$ROOT/.venv/bin/python" -m pip install --timeout 120 --no-deps lerobot==0.4.3
"$ROOT/.venv/bin/python" -m pip install --timeout 120 'datasets>=4.0,<4.2' 'huggingface-hub>=0.34.2,<0.36' 'av>=15,<16' 'accelerate>=1.10,<2' 'jsonlines>=4,<5' 'packaging>=24.2,<26' draccus==0.10.0 'einops>=0.8,<0.9' 'deepdiff>=7,<9' 'imageio>=2.34,<3' scipy numpy==2.2.6
