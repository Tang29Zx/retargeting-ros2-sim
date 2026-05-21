#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "${WORKSPACE_DIR}/scripts/setup_ros_env.sh"
setup_ros_env
set -u

ros2 run human_pose_processor processor_node
