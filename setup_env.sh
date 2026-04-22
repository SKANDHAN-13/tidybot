#!/usr/bin/env bash
# setup_env.sh
# ============
# Sources the complete ROS 2 + TidyBot workspace environment.
#
# Usage (run once per terminal, or add both lines to ~/.bashrc):
#
#   source /opt/ros/humble/setup.bash          # ROS 2 Humble base
#   source ~/tidybot_ws/install/setup.bash     # TidyBot overlay
#
# Or simply:
#   source ~/tidybot_ws/src/setup_env.sh
#
# Environment captured from the development machine:
#   OS           : Ubuntu 22.04.5 LTS
#   ROS_DISTRO   : humble
#   Gazebo       : 11.x (Classic)
#   Python       : 3.10
#   RMW          : rmw_cyclonedds_cpp  (recommended for Nav2)

# ── 1. ROS 2 Humble base install ────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[setup_env] ERROR: /opt/ros/humble/setup.bash not found."
    echo "            Install ROS 2 Humble first:"
    echo "            https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    return 1
fi

# ── 2. TidyBot workspace overlay ────────────────────────────────────────────
TIDYBOT_WS="${HOME}/tidybot_ws"
if [ -f "${TIDYBOT_WS}/install/setup.bash" ]; then
    source "${TIDYBOT_WS}/install/setup.bash"
else
    echo "[setup_env] WARNING: ${TIDYBOT_WS}/install/setup.bash not found."
    echo "            Run 'colcon build' inside ${TIDYBOT_WS} first."
fi

# ── 3. RMW — CycloneDDS (recommended for Nav2 reliability) ──────────────────
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── 4. Gazebo Classic model / plugin paths ───────────────────────────────────
# Tells Gazebo where to find the TidyBot models (e.g. world objects).
GAZEBO_MODEL_DIRS="${TIDYBOT_WS}/src/tidybot_gazebo/models"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_DIRS}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

# ── 5. Python path (in case pip-installed packages are not on path) ──────────
# Only needed if you installed deps with --user or in a virtualenv
# export PYTHONPATH="${HOME}/.local/lib/python3.10/site-packages:${PYTHONPATH}"

# ── 6. Optional: suppress RViz Wayland warning on Gnome+Wayland ─────────────
# Uncomment if RViz fails to open under Wayland:
# export QT_QPA_PLATFORM=xcb

echo "[setup_env] ROS_DISTRO        = ${ROS_DISTRO}"
echo "[setup_env] RMW_IMPLEMENTATION= ${RMW_IMPLEMENTATION}"
echo "[setup_env] AMENT_PREFIX_PATH (first entry) = $(echo $AMENT_PREFIX_PATH | cut -d: -f1)"
echo "[setup_env] Ready. Launch with:"
echo "            ros2 launch tidybot_bringup tidybot.launch.py"
