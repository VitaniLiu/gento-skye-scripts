#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CHASSIS="$ROOT/src/teleop/chassis"
export PYTHONDONTWRITEBYTECODE=1
mode="${1:-offline}"
if [ "$#" -gt 0 ]; then shift; fi
case "$mode" in
 offline)
  exec python3 -B "$CHASSIS/src/teleop/sender.py" --help "$@" ;;
 teleop) exec bash "$CHASSIS/teleop.sh" "$@" ;;
 check) exec bash "$CHASSIS/check.sh" "$@" ;;
 preview) exec python3 -B "$CHASSIS/src/teleop/preview.py" "$@" ;;
 *) echo 'Usage: chassis.sh [offline|teleop|check|preview]' >&2; exit 2 ;;
esac
