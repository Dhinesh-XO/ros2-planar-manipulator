#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$KINESHIA_ROOT"
python /usr/bin/colcon build --symlink-install \
    --cmake-args -DPython3_EXECUTABLE="$KINESHIA_ROOT/.venv/bin/python" \
    -DPYTHON_EXECUTABLE="$KINESHIA_ROOT/.venv/bin/python"
