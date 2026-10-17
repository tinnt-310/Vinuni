#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: Khong tim thay ROS 2 Humble tai /opt/ros/humble." >&2
    exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u
exec /usr/bin/python3 "${PROJECT_DIR}/scripts/manual_keyboard.py" "$@"
