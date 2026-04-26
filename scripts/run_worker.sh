#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONDA_ENV="${HYBRIK_CONDA_ENV:-pjHybrik-cu130}"

if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base)"
elif [ -d "$HOME/miniconda3" ]; then
    CONDA_BASE="$HOME/miniconda3"
elif [ -d "$HOME/anaconda3" ]; then
    CONDA_BASE="$HOME/anaconda3"
elif [ -d "$HOME/miniforge3" ]; then
    CONDA_BASE="$HOME/miniforge3"
else
    echo "Could not find conda. Activate ${CONDA_ENV} manually and run the worker script directly." >&2
    exit 1
fi

set +u
source "${CONDA_BASE}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}"
set -u

python "${WORKSPACE_DIR}/src/hybrik_ros/scripts/hybrik_worker_server.py"
