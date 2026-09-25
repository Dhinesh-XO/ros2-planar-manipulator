#!/usr/bin/env bash
# Source this script in each terminal; do not execute it as a child shell.
KINESHIA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$KINESHIA_ROOT/.venv/bin/activate"
export PATH="$KINESHIA_ROOT/.venv/bin:/usr/bin:/bin:$PATH"
export ROS_DOMAIN_ID="${KINESHIA_ROS_DOMAIN_ID:-67}"
export ROS_LOCALHOST_ONLY="${KINESHIA_LOCALHOST_ONLY:-1}"
if [ -d "$KINESHIA_ROOT/.deps/gz_harmonic/opt/ros/humble" ]; then
    KINESHIA_GZ_PREFIX="$KINESHIA_ROOT/.deps/gz_harmonic/opt/ros/humble"
    export AMENT_PREFIX_PATH="$KINESHIA_GZ_PREFIX:$AMENT_PREFIX_PATH"
    export LD_LIBRARY_PATH="$KINESHIA_GZ_PREFIX/lib:$LD_LIBRARY_PATH"
    export PYTHONPATH="$KINESHIA_GZ_PREFIX/local/lib/python3.10/dist-packages:$PYTHONPATH"
fi
if [ -f "$KINESHIA_ROOT/install/setup.bash" ]; then
    source "$KINESHIA_ROOT/install/setup.bash"
fi
