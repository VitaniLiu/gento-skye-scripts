#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
if [ -r /etc/apex/apex_ros_env.sh ]; then
    source /etc/apex/apex_ros_env.sh >&2
elif [ -r /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash >&2
    if [ -r /opt/kernelmind/apex/install/setup.bash ]; then source /opt/kernelmind/apex/install/setup.bash >&2; fi
else
    echo "ROS environment unavailable" >&2
    exit 1
fi
export ROS_DOMAIN_ID="${GENTO_CHASSIS_DOMAIN_ID:-24}"
# Use a chassis-local discovery profile without changing Apex defaults.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PROJECT_DIR/config/cyclonedds.xml"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_LOG_DIR="$(PYTHONPATH="$PROJECT_DIR/src/common" python3 -B -c 'from project_paths import output_directory; print(output_directory("logs") / "ros")')"
export RCUTILS_LOGGING_USE_STDOUT=0
mkdir -p -- "$ROS_LOG_DIR"
cd -- "$PROJECT_DIR"
exec python3 -u -B "$PROJECT_DIR/src/robot/robot_endpoint.py" "$@"
