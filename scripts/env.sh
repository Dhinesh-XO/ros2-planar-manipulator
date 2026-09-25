#!/usr/bin/env bash
# Source this script in each terminal; do not execute it as a child shell.
KINESHIA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$KINESHIA_ROOT/.venv/bin/activate"
export PATH="$KINESHIA_ROOT/.venv/bin:/usr/bin:/bin:$PATH"
export ROS_DOMAIN_ID="${KINESHIA_ROS_DOMAIN_ID:-67}"
export ROS_LOCALHOST_ONLY=1
if [ -f "$KINESHIA_ROOT/install/setup.bash" ]; then
    source "$KINESHIA_ROOT/install/setup.bash"
fi
