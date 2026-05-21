#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "${WORKSPACE_DIR}/scripts/setup_ros_env.sh"
setup_ros_env
set -u

ros2 run hybrik_pose_bridge pose_array_bridge
