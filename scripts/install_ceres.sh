#!/usr/bin/env bash
set -euo pipefail
OVERLAY="${HOME}/ros_overlay"
DEBDIR="/tmp/slam_debs"
mkdir -p "$DEBDIR" "$OVERLAY"
cd "$DEBDIR"
URL="http://archive.ubuntu.com/ubuntu/pool/universe/c/ceres-solver/libceres2_2.0.0+dfsg1-5_amd64.deb"
wget -q -c "$URL" -O libceres2.deb || curl -fsSL "$URL" -o libceres2.deb
dpkg-deb -x libceres2.deb "$OVERLAY"
echo "ceres libs:"
find "$OVERLAY/usr/lib" -name 'libceres*' 2>/dev/null
ldd "$OVERLAY/opt/ros/humble/lib/libceres_solver_plugin.so" 2>/dev/null | grep 'not found' || echo 'plugin libs OK'
