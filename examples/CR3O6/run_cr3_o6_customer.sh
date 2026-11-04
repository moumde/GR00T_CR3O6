#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

PROJECT_PYTHON="$REPO_ROOT/.venv/bin/python"
LEROBOT_PYTHON="/home/je/anaconda3/envs/lerobot/bin/python"

if [[ -x "$PROJECT_PYTHON" ]] && "$PROJECT_PYTHON" -c "import lerobot, pyrealsense2, pymodbus, serial" >/dev/null 2>&1; then
    RUNTIME_PYTHON="$PROJECT_PYTHON"
    PYTHON_RUNNER=(uv run --project "$REPO_ROOT" --no-sync python)
elif [[ -x "$LEROBOT_PYTHON" ]] && "$LEROBOT_PYTHON" -c "import lerobot, pyrealsense2, pymodbus, serial" >/dev/null 2>&1; then
    RUNTIME_PYTHON="$LEROBOT_PYTHON"
    PYTHON_RUNNER=("$LEROBOT_PYTHON")
else
    echo "Missing CR3/O6 runtime dependencies: lerobot, pyrealsense2, pymodbus and pyserial." >&2
    echo "Install the LinkerHand requirements and LeRobot Intel RealSense extra." >&2
    exit 1
fi

PYTHON_LIB_DIR="$("$RUNTIME_PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR") or "")')"
if [[ -n "$PYTHON_LIB_DIR" && -d "$PYTHON_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$PYTHON_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

exec "${PYTHON_RUNNER[@]}" "$SCRIPT_DIR/deploy_cr3_o6_customer.py" "$@"
