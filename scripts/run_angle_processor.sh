#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "${WORKSPACE_DIR}/scripts/setup_ros_env.sh"
setup_ros_env
set -u

ros2 run angle_processor angle_processor --ros-args \
    -p input_topic:=/human_joint_angles \
    -p output_topic:=/human_joint_angles_filtered \
    -p max_jump_deg:=20.0 \
    -p moving_average_window:=5
