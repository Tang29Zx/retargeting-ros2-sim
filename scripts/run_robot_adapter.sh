#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "${WORKSPACE_DIR}/scripts/setup_ros_env.sh"
setup_ros_env
set -u

ros2 run robot_adapter robot_adapter --ros-args \
    -p input_topic:=/human_joint_angles_filtered \
    -p output_topic:=/g1_joint_targets \
    -p max_step_rad:=0.2
