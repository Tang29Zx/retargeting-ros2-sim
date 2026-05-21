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
pkill -f "ros2 run robot_adapter robot_adapter" || true
pkill -f hybrik_worker_server.py || true
tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true

sleep 1

tmux new-session -d -s "${SESSION_NAME}" -n wk \
    "echo '[wk] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_worker.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:wk" -n hybrik \
    "echo '[hybrik] waiting for worker'; sleep 4; echo '[hybrik] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_ros_node.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:hybrik" -n pose \
    "echo '[pose] waiting for ROS node'; sleep 6; echo '[pose] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_human_pose_processor.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:pose" -n key \
    "echo '[key] waiting for processor'; sleep 8; echo '[key] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_keypoint_validator.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:key" -n rb \
    "echo '[rb] waiting for keypoints'; sleep 9; echo '[rb] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_retarget_keypoint_bridge.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:rb" -n fit \
    "echo '[fit] waiting for keypoints'; sleep 10; echo '[fit] starting'; cd '${WORKSPACE_DIR}' && bash ./scripts/run_human_model_fitter.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:fit" -n ang \
    "echo '[ang] waiting for fitted angles'; sleep 12; echo '[ang] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_angle_processor.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:ang" -n rob \
    "echo '[rob] waiting for filtered angles'; sleep 14; echo '[rob] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_robot_adapter.sh; exec bash"

tmux new-window -a -t "${SESSION_NAME}:rob" -n pb \
    "echo '[pb] waiting for processor'; sleep 16; echo '[pb] starting'; cd '${WORKSPACE_DIR}' && ./scripts/run_human_pose_bridge.sh; exec bash"

tmux select-window -t "${SESSION_NAME}:wk"

if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "${SESSION_NAME}"
else
    tmux attach-session -t "${SESSION_NAME}"
fi
