#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$KINESHIA_ROOT"
mkdir -p artifacts
python -m planar_arm_control.controller_node > artifacts/demo_controller.log 2>&1 &
KINESHIA_DEMO_PID=$!
trap 'kill -INT "$KINESHIA_DEMO_PID" 2>/dev/null || true; wait "$KINESHIA_DEMO_PID" 2>/dev/null || true' EXIT
python -m planar_arm_control.gui_node --demo --record "${1:-artifacts/demo.mp4}"
