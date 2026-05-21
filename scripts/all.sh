#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION_NAME="hybrik"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed. Install it first, for example: sudo apt install tmux" >&2
    exit 1
fi

pkill -f "ros2 run hybrik_ros hybrik_node" || true
pkill -f "ros2 run hybrik_pose_bridge pose_array_bridge" || true
pkill -f "ros2 run human_pose_processor processor_node" || true
pkill -f "ros2 run human_pose_processor keypoint_validator" || true
pkill -f "ros2 run human_model_fitter fitter_node" || true
pkill -f "ros2 run angle_processor angle_processor" || true
pkill -f hybrik_worker_server.py || true
tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true

sleep 1

tmux new-session -d -s "${SESSION_NAME}" -n worker \
    "echo '[worker] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_worker.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n ros-node \
    "echo '[ros node] waiting for worker'; sleep 4; echo '[ros node] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_ros_node.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n human-processor \
    "echo '[human processor] waiting for ROS node'; sleep 6; echo '[human processor] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_human_pose_processor.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n keypoint-validator \
    "echo '[keypoint validator] waiting for processor'; sleep 8; echo '[keypoint validator] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_keypoint_validator.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n model-fitter \
    "echo '[model fitter] waiting for keypoints'; sleep 10; echo '[model fitter] starting'; cd '${WORKSPACE_DIR}' && bash ./scripts/run_human_model_fitter.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n angle-processor \
    "echo '[angle processor] waiting for fitted angles'; sleep 12; echo '[angle processor] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_angle_processor.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n human-bridge \
    "echo '[human bridge] waiting for processor'; sleep 14; echo '[human bridge] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_human_pose_bridge.sh; exec bash"

tmux select-window -t "${SESSION_NAME}:worker"

if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "${SESSION_NAME}"
else
    tmux attach-session -t "${SESSION_NAME}"
fi
