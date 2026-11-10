#!/usr/bin/env bash
set -euo pipefail
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)"
mode="--check"
if [ "$#" -gt 0 ]; then mode="$1"; fi
if [ "$mode" = "--check" ]; then
    exec python3 -B -c 'import json,sys; p=json.load(open(sys.argv[1])); print(json.dumps({"configuration":p,"motion_started":False,"service_restarted":False}))' "$ROOT/configs/teleop/gripper-slow.json"
fi
if [ "$mode" != "--execute" ]; then
    echo "Usage: gripper-slow.sh [--check|--execute]" >&2
    exit 2
fi
# ROS-generated setup scripts reference optional unset variables.
set +u
source /etc/apex/apex_ros_env.sh
source /opt/kernelmind/apex_tool/install/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch "$ROOT/deployment/robot/gripper-slow.launch.py"
