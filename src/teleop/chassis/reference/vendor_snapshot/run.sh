#!/bin/bash
source /etc/apex/apex_ros_env.sh
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
exec python3 /home/user/gamepad_control/receiver.py
