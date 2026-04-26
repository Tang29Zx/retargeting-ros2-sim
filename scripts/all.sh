#!/usr/bin/env bash
set -e

WORKSPACE_DIR="/home/tang/robotics/workspaces/hybrik-ros2-sim"

pkill -f "ros2 run hybrik_ros hybrik_node" || true
pkill -f hybrik_worker_server.py || true

sleep 1

gnome-terminal -- bash -lc "cd ${WORKSPACE_DIR} && ./scripts/run_worker.sh; exec bash"

sleep 5

gnome-terminal -- bash -lc "cd ${WORKSPACE_DIR} && ./scripts/run_ros_node.sh; exec bash"
