#!/usr/bin/env bash

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

deactivate_conda_for_ros() {
    if [ -z "${CONDA_DEFAULT_ENV:-}" ] && [ -z "${CONDA_PREFIX:-}" ]; then
        return
    fi

    local conda_base=""
    if [ -n "${CONDA_EXE:-}" ]; then
        conda_base="$(dirname "$(dirname "${CONDA_EXE}")")"
    elif command -v conda >/dev/null 2>&1; then
        conda_base="$(conda info --base)"
    elif [ -d "$HOME/miniconda3" ]; then
        conda_base="$HOME/miniconda3"
    elif [ -d "$HOME/anaconda3" ]; then
        conda_base="$HOME/anaconda3"
    elif [ -d "$HOME/miniforge3" ]; then
        conda_base="$HOME/miniforge3"
    fi

    if [ -n "${conda_base}" ] && [ -f "${conda_base}/etc/profile.d/conda.sh" ]; then
        # Match run_worker.sh style: load conda shell support before changing envs.
        source "${conda_base}/etc/profile.d/conda.sh"
        while [ "${CONDA_SHLVL:-0}" -gt 0 ]; do
            conda deactivate
        done
        echo "Deactivated conda for ROS Jazzy."
        return
    fi

    echo "Warning: conda env '${CONDA_DEFAULT_ENV:-unknown}' is active, but conda.sh was not found." >&2
    echo "ROS Jazzy may import the wrong Python packages." >&2
}

setup_ros_env() {
    deactivate_conda_for_ros

    source "${ROS_SETUP}"

    cd "${WORKSPACE_DIR}"
    source "${WORKSPACE_DIR}/install/setup.bash"
}
