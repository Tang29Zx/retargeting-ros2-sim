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
pkill -f hybrik_worker_server.py || true
tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true

sleep 1

tmux new-session -d -s "${SESSION_NAME}" -n worker \
    "echo '[worker] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_worker.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n ros-node \
    "echo '[ros node] waiting for worker'; sleep 4; echo '[ros node] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_ros_node.sh; exec bash"

tmux new-window -t "${SESSION_NAME}" -n pose-bridge \
    "echo '[pose bridge] waiting for ROS node'; sleep 6; echo '[pose bridge] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_pose_bridge.sh; exec bash"

tmux select-window -t "${SESSION_NAME}:worker"

if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "${SESSION_NAME}"
else
    tmux attach-session -t "${SESSION_NAME}"
fi
