#!/usr/bin/env bash
set -e

# Setup overlay environment and model paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_env.sh"

PX4_DIR="/home/hasan-fashtouk/Documents/PX4-Autopilot"
cd "${PX4_DIR}"

export GIT_SUBMODULES_ARE_EVIL=1

# Optional world argument and optional --gui flag
WORLD="default"
HEADLESS=1

for arg in "$@"; do
    case "$arg" in
        --gui)
            HEADLESS=""
            ;;
        --headless)
            HEADLESS=1
            ;;
        *)
            WORLD="${arg%.*}"
            ;;
    esac
done

export HEADLESS

# Automatically sync world symlinks from px4_sando_bridge into PX4 simulation worlds
BRIDGE_WORLDS_DIR="${SCRIPT_DIR}/src/px4_sando_bridge/worlds"
PX4_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
mkdir -p "${PX4_WORLDS_DIR}"
for w in "${BRIDGE_WORLDS_DIR}"/*.sdf; do
    if [ -f "$w" ]; then
        fname="$(basename "$w")"
        ln -sf "$w" "${PX4_WORLDS_DIR}/${fname}"
    fi
done

# Automatically sync model symlinks from px4_sando_bridge into PX4 simulation models
BRIDGE_MODELS_DIR="${SCRIPT_DIR}/src/px4_sando_bridge/models"
PX4_MODELS_DIR="${PX4_DIR}/Tools/simulation/gz/models"
mkdir -p "${PX4_MODELS_DIR}"
for m in "${BRIDGE_MODELS_DIR}"/*; do
    if [ -d "$m" ] || [ -L "$m" ]; then
        mname="$(basename "$m")"
        ln -sfn "$m" "${PX4_MODELS_DIR}/${mname}"
    fi
done

# Ensure no stale PX4 or Gazebo instances are lingering in the background
echo "Cleaning up any previous PX4 / Gazebo instances..."
killall -9 px4 gz-sim-main 2>/dev/null || true
sleep 1

echo "=========================================================="
if [ -n "${HEADLESS}" ]; then
    echo " Starting PX4 SITL Headless (No Gazebo GUI) (World: ${WORLD})"
else
    echo " Starting PX4 SITL with Gazebo GUI (World: ${WORLD})"
fi
echo "=========================================================="

if [ "${WORLD}" != "default" ]; then
    PX4_GZ_WORLD="${WORLD}" make px4_sitl gz_x500_lidar_3d
else
    make px4_sitl gz_x500_lidar_3d
fi
