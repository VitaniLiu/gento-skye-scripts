#!/usr/bin/env bash
set -eo pipefail
source /etc/apex/apex_ros_env.sh
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch "$ROOT/deployment/robot/waist-lock.launch.py"
