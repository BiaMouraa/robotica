#!/usr/bin/env bash
# Source este arquivo dentro do WSL antes de colcon/launch.
set +u
source /opt/ros/humble/setup.bash
# overlay extraido sem sudo (robot_localization + deps)

OVERLAY="$HOME/ros_overlay/opt/ros/humble"
export AMENT_PREFIX_PATH="$OVERLAY:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$OVERLAY:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$OVERLAY/lib:$HOME/ros_overlay/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PATH="$OVERLAY/bin:${PATH:-}"

if [ -d "$OVERLAY/local/lib/python3.10/dist-packages" ]; then
  export PYTHONPATH="$OVERLAY/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
fi
if [ -d "$OVERLAY/lib/python3.10/site-packages" ]; then
  export PYTHONPATH="$OVERLAY/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
