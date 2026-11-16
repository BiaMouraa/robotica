#!/usr/bin/env bash
set -euo pipefail
OVERLAY="${HOME}/ros_overlay"
DEBDIR="/tmp/slam_debs"
BASE="http://packages.ros.org/ros2/ubuntu"
mkdir -p "$DEBDIR" "$OVERLAY"
cd "$DEBDIR"

FILES=(
  "pool/main/r/ros-humble-slam-toolbox/ros-humble-slam-toolbox_2.6.10-1jammy.20261110.140327_amd64.deb"
  "pool/main/r/ros-humble-nav2-map-server/ros-humble-nav2-map-server_1.1.20-1jammy.20261110.104635_amd64.deb"
  "pool/main/r/ros-humble-nav2-msgs/ros-humble-nav2-msgs_1.1.20-1jammy.20261110.094659_amd64.deb"
  "pool/main/r/ros-humble-nav2-util/ros-humble-nav2-util_1.1.20-1jammy.20261110.103706_amd64.deb"
)

for f in "${FILES[@]}"; do
  name=$(basename "$f")
  if [[ ! -f "$name" ]]; then
    echo "GET $name"
    wget -q "$BASE/$f" -O "$name" || curl -fsSL "$BASE/$f" -o "$name"
  fi
done

for deb in *.deb; do
  echo "EXTRACT $deb"
  dpkg-deb -x "$deb" "$OVERLAY"
done

echo "--- slam_toolbox ---"
ls "$OVERLAY/opt/ros/humble/lib/slam_toolbox" || true
ls "$OVERLAY/opt/ros/humble/share/ament_index/resource_index/packages" | grep -E 'slam|nav2' || true
