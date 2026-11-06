#!/usr/bin/env bash
set -euo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11990}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
exec ros2 launch coal_platform_bridge coal_face_interfaces.launch.py "$@"
