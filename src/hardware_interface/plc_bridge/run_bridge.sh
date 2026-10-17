#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="${PROJECT_DIR}/.venv/bin/python"
SYSTEM_PYTHON="/usr/bin/python3"

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "ERROR: Chua co .venv. Chay: python3 -m venv .venv && .venv/bin/python -m pip install -e ." >&2
    exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: Khong tim thay ROS 2 Humble tai /opt/ros/humble." >&2
    exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u
VENV_SITE="$("${VENV_PYTHON}" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${VENV_SITE}:${PYTHONPATH:-}"
exec "${SYSTEM_PYTHON}" "${PROJECT_DIR}/scripts/open_loop_bridge.py" \
    --config "${PROJECT_DIR}/config/default.yaml" \
    --bridge-config "${PROJECT_DIR}/config/open_loop_bridge.yaml" \
    "$@"
