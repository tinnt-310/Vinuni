#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "ERROR: Chua co moi truong .venv." >&2
    echo "Cai mot lan bang cac lenh:" >&2
    echo "  cd ${PROJECT_DIR}" >&2
    echo "  python3 -m venv .venv" >&2
    echo "  .venv/bin/python -m pip install -e ." >&2
    exit 2
fi

exec "${PYTHON_BIN}" "${PROJECT_DIR}/scripts/interactive_cli.py" \
    --config "${PROJECT_DIR}/config/default.yaml" \
    --execute \
    "$@"
