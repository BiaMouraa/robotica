#!/usr/bin/env bash
set +u
REPO="/mnt/c/Users/MatheusMendonca/Desktop/robotica"
source "$REPO/scripts/env_wsl.sh"
source "$HOME/pratica5_ws/install/setup.bash" 2>/dev/null || true

echo "=== matando processos antigos ==="
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f async_slam 2>/dev/null || true
pkill -9 -f localization_slam 2>/dev/null || true
pkill -9 -f gt_odom 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f teleop_twist 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f spawn_entity 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f xterm 2>/dev/null || true
sleep 2

rsync -a --delete --exclude '.git' --exclude 'scripts' "$REPO/" "$HOME/pratica5_ws/src/meu_robo_description/"
cd "$HOME/pratica5_ws"
colcon build --packages-select meu_robo_description --symlink-install
source "$HOME/pratica5_ws/install/setup.bash"

export GAZEBO_MODEL_DATABASE_URI=""
export DISPLAY="${DISPLAY:-:0}"

SCENARIO="${1:-gt}"
LOGDIR="/tmp/slam_run_$$"
mkdir -p "$LOGDIR"

echo "=== 1/3 Gazebo + RViz (scenario=$SCENARIO) ==="
ros2 launch meu_robo_description slam_sim.launch.py \
  scenario:="$SCENARIO" gui:=true rviz:=true \
  >"$LOGDIR/sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 90); do
  if grep -q "Successfully spawned" "$LOGDIR/sim.log" 2>/dev/null; then
    echo "spawn OK (${i}s)"
    break
  fi
  if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "ERRO: simulacao morreu"
    tail -50 "$LOGDIR/sim.log"
    exit 1
  fi
  sleep 1
done
sleep 4

echo "=== 2/3 SLAM mapping ==="
ros2 launch meu_robo_description slam.launch.py mode:=mapping \
  >"$LOGDIR/slam.log" 2>&1 &
SLAM_PID=$!
sleep 5

echo "=== 3/3 movimento de teste (5s) + teleop ==="
# prova que o robo responde a cmd_vel
timeout 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25}, angular: {z: 0.4}}" --rate 10 >/dev/null 2>&1 || true
# para o robo
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true

# teleop em xterm com fonte padrao (evita erro de font)
if command -v xterm >/dev/null 2>&1; then
  xterm -fa Monospace -fs 12 -T "TELEOP - clique aqui e aperte i" \
    -e bash -lc "source $REPO/scripts/env_wsl.sh; source $HOME/pratica5_ws/install/setup.bash; echo 'Clique nesta janela'; echo 'i=frente  ,=tras  j/l=girar  k=parar'; echo; ros2 run teleop_twist_keyboard teleop_twist_keyboard" &
  TELEOP_PID=$!
else
  echo "Sem xterm: rode manualmente: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
  TELEOP_PID=0
fi

echo
echo "============================================"
echo " PRONTO"
echo " 1) Clique no Gazebo/RViz para ver o robo"
echo " 2) Clique na janela preta TELEOP"
echo " 3) Aperte: i  (frente)   j/l (girar)   k (parar)"
echo " 4) No RViz o mapa (/map) vai preenchendo"
echo " Logs: $LOGDIR"
echo "============================================"

wait $SIM_PID
