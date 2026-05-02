#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    echo "Warning: currently inside conda env '${CONDA_DEFAULT_ENV}'."
    echo "ROS Jazzy should normally run outside conda. Run 'conda deactivate' first if import errors appear."
fi

set +u
source "${ROS_SETUP}"

cd "${WORKSPACE_DIR}"
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

ros2 run hybrik_pose_bridge pose_array_bridge --ros-args \
    -p input_topic:=/hybrik_pose_world \
    -p output_topic:=/human_pose_points
