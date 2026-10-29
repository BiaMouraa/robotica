#!/usr/bin/env bash
set -euo pipefail

OVERLAY="$HOME/ros_overlay"
DEBDIR="/tmp/rl_debs"
WS="$HOME/pratica5_ws"
SRC_WIN="/mnt/c/Users/MatheusMendonca/Desktop/robotica"

mkdir -p "$DEBDIR" "$OVERLAY" "$WS/src"

cd "$DEBDIR"
apt-get download \
  ros-humble-robot-localization \
  ros-humble-diagnostic-updater \
  ros-humble-geographic-msgs \
  libgeographic19

for deb in *.deb; do
  echo "EXTRACT $deb"
  dpkg-deb -x "$deb" "$OVERLAY"
done

echo "--- overlay packages ---"
ls "$OVERLAY/opt/ros/humble/share" || true
ls "$OVERLAY/opt/ros/humble/lib/robot_localization" || true
find "$OVERLAY" -name 'ekf_node' -o -name 'libGeographic*' | head

# Copia o pacote do Windows para o workspace Linux (ext4, colcon mais estavel)
rsync -a --delete --exclude '.git' --exclude 'scripts' "$SRC_WIN/" "$WS/src/meu_robo_description/"

echo "--- package copied ---"
ls "$WS/src/meu_robo_description"
