#!/usr/bin/env bash
# Environment setup script for px4_sando_ws overlay workspace

PX4_SANDO_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# Source underlay workspace (sando_lyrical_ws), which also sources /opt/ros/lyrical and configures Gurobi
if [ -f "/home/hasan-fashtouk/code/sando_lyrical_ws/setup_env.sh" ]; then
  source /home/hasan-fashtouk/code/sando_lyrical_ws/setup_env.sh
elif [ -f "/opt/ros/lyrical/setup.bash" ]; then
  source /opt/ros/lyrical/setup.bash
else
  echo "[px4_sando_ws] Warning: /opt/ros/lyrical/setup.bash not found"
fi

# Ensure system python takes precedence over Gurobi's bundled python
export PATH="/usr/local/bin:/usr/bin:${PATH}"

# Source overlay workspace if built
if [ -f "${PX4_SANDO_WS_DIR}/install/setup.bash" ]; then
  source "${PX4_SANDO_WS_DIR}/install/setup.bash"
fi

# Ensure compatibility libraries (such as libtinyxml2 for MicroXRCEAgent) are accessible
if [ -d "/home/hasan-fashtouk/code/lib" ]; then
  export LD_LIBRARY_PATH="/home/hasan-fashtouk/code/lib:${LD_LIBRARY_PATH}"
fi

# GZ Resource paths if custom models or worlds exist
BRIDGE_SHARE="${SCRIPT_DIR}/src/px4_sando_bridge"
if [ -d "${BRIDGE_SHARE}/models" ]; then
  export GZ_SIM_RESOURCE_PATH="${BRIDGE_SHARE}/models:${BRIDGE_SHARE}/worlds:${GZ_SIM_RESOURCE_PATH}"
fi
