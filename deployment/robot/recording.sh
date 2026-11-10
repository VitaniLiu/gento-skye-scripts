#!/bin/bash
# Optional robot-local entrypoint for an installed project checkout. No services changed.
set -euo pipefail
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
source /etc/apex/apex_ros_env.sh >&2
export PYTHONDONTWRITEBYTECODE=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
exec python3 -B -u -c '
import pathlib,sys
root=pathlib.Path(sys.argv.pop(1))
source="\n".join((root/"src/recording"/name).read_text() for name in ("protocol.py","transport.py","worker.py"))
exec(compile(source,"robot_recording_bundle","exec"))
' "$ROOT" "$@"
