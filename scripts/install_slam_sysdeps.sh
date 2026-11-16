#!/usr/bin/env bash
set -euo pipefail
OVERLAY="${HOME}/ros_overlay"
DEBDIR="/tmp/slam_sysdeps"
BASE="http://archive.ubuntu.com/ubuntu"
mkdir -p "$DEBDIR" "$OVERLAY"
cd "$DEBDIR"

FILES=(
  "pool/universe/g/google-glog/libgoogle-glog0v5_0.5.0+really0.4.0-2_amd64.deb"
  "pool/universe/s/suitesparse/libspqr2_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/s/suitesparse/libcholmod3_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/s/suitesparse/libcxsparse3_5.10.1+dfsg-4build1_amd64.deb"
  "pool/main/s/suitesparse/libsuitesparseconfig5_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/s/suitesparse/libamd2_5.10.1+dfsg-4build1_amd64.deb"
  "pool/main/s/suitesparse/libcolamd2_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/s/suitesparse/libcamd2_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/s/suitesparse/libccolamd2_5.10.1+dfsg-4build1_amd64.deb"
  "pool/universe/m/metis/libmetis5_5.1.0.dfsg-7build2_amd64.deb"
  "pool/universe/g/gflags/libgflags2.2_2.2.2-2_amd64.deb"
)

for f in "${FILES[@]}"; do
  name=$(basename "$f")
  if [[ ! -f "$name" ]]; then
    echo "GET $name"
    wget -q "$BASE/$f" -O "$name" || curl -fsSL "$BASE/$f" -o "$name"
  fi
  dpkg-deb -x "$name" "$OVERLAY"
done

export LD_LIBRARY_PATH="$OVERLAY/usr/lib:$OVERLAY/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/home/matheusendonca/ros_overlay/opt/ros/humble/lib:$LD_LIBRARY_PATH"
ldd /home/matheusendonca/ros_overlay/opt/ros/humble/lib/libceres_solver_plugin.so | grep 'not found' || echo 'plugin OK'
