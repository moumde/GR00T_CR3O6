#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PROJECT_PYTHON="$REPO_ROOT/.venv/bin/python"
LEROBOT_PYTHON="/home/je/anaconda3/envs/lerobot/bin/python"

if [[ -x "$PROJECT_PYTHON" ]] && "$PROJECT_PYTHON" -c "import lerobot, pyrealsense2, pymodbus, serial, aiohttp, PIL, yaml, cv2" >/dev/null 2>&1; then
    RUNTIME_PYTHON="$PROJECT_PYTHON"
elif [[ -x "$LEROBOT_PYTHON" ]] && "$LEROBOT_PYTHON" -c "import lerobot, pyrealsense2, pymodbus, serial, aiohttp, PIL, yaml, cv2" >/dev/null 2>&1; then
    RUNTIME_PYTHON="$LEROBOT_PYTHON"
elif command -v python >/dev/null 2>&1 && python -c "import sys; assert sys.version_info >= (3, 11); import lerobot, pyrealsense2, pymodbus, serial, aiohttp, PIL, yaml, cv2" >/dev/null 2>&1; then
    RUNTIME_PYTHON="$(command -v python)"
else
    echo "No Python 3.11+ environment with the CR3/O6, camera and Customer API dependencies was found." >&2
    echo "Activate the already validated robot environment or set up its dependencies." >&2
    exit 1
fi

PYTHON_LIB_DIR="$("$RUNTIME_PYTHON" -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR") or "")')"
if [[ -n "$PYTHON_LIB_DIR" && -d "$PYTHON_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$PYTHON_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

exec "$RUNTIME_PYTHON" "$SCRIPT_DIR/stationary_client_redeploy_auto.py" \
    --config "$SCRIPT_DIR/config-v2.toml" "$@"
