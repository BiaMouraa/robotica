# Prática 05.2 — SLAM

Odometria **relativa** apenas (sem GPS/EKF). Pacote: `slam_toolbox`.

## Dependências (WSL)

Preferível (com senha sudo):

```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  libceres2 libgoogle-glog0v5
```

Sem sudo, use os scripts do repositório (overlay em `~/ros_overlay`):

```bash
bash /mnt/c/Users/MatheusMendonca/Desktop/robotica/scripts/install_slam_toolbox.sh
bash /mnt/c/Users/MatheusMendonca/Desktop/robotica/scripts/install_ceres.sh
bash /mnt/c/Users/MatheusMendonca/Desktop/robotica/scripts/install_slam_sysdeps.sh
source /mnt/c/Users/MatheusMendonca/Desktop/robotica/scripts/env_wsl.sh
```

Build do pacote:

```bash
source /mnt/c/Users/MatheusMendonca/Desktop/robotica/scripts/env_wsl.sh
rsync -a --delete --exclude '.git' --exclude 'scripts' \
  /mnt/c/Users/MatheusMendonca/Desktop/robotica/ ~/pratica5_ws/src/meu_robo_description/
cd ~/pratica5_ws && colcon build --packages-select meu_robo_description --symlink-install
source install/setup.bash
```

## Exercício 1.1 — Ruído no Lidar 2D

O Lidar publica **`/scan`** (`sensor_msgs/LaserScan`, 360°) com ruído gaussiano
controlado por `lidar_noise_scale` (URDF). Escala `0.0` = sem ruído; `5.0` = forte.

## Exercício 2.1 / 2.2 — slam_toolbox (3 cenários)

Abra **3 terminais** WSL (todos com `source` do ROS + workspace).

### Terminal 1 — simulação

| Cenário | Comando |
|--------|---------|
| Sem ruído (tudo GT) | `ros2 launch meu_robo_description slam_sim.launch.py scenario:=gt` |
| Ruído só no Lidar + odom GT | `ros2 launch meu_robo_description slam_sim.launch.py scenario:=lidar_noise` |
| Ruído no Lidar e na odometria | `ros2 launch meu_robo_description slam_sim.launch.py scenario:=both_noise` |

### Terminal 2 — SLAM (mapping)

```bash
ros2 launch meu_robo_description slam.launch.py mode:=mapping
```

### Terminal 3 — teleop

```bash
ros2 launch meu_robo_description teleop.launch.py
```

Explore a sala até o mapa no RViz (`/map`) ficar consistente. Compare visualmente
os três cenários: com odometria ruim o mapa distorce mais até o loop closure.

### Salvar o mapa (pose-graph do slam_toolbox)

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/tmp/meu_robo_slam_map'}}"
```

## Exercício 3.1 — Robô sequestrado

1. Rode o cenário **`both_noise`**, mapeie e **salve** o mapa (acima).
2. Pare o nó de mapping. Suba de novo a simulação + localização:

```bash
# Terminal 1
ros2 launch meu_robo_description slam_sim.launch.py scenario:=both_noise

# Terminal 2
ros2 launch meu_robo_description slam.launch.py mode:=localization
```

3. No Gazebo: **Pause** → selecione o robô (modo translação) → arraste para outro
   lado do mapa → **Play**.
4. Observe no RViz se a pose no mapa se **recupera** (scan matching / localization).

## Tópicos importantes

| Tópico | Papel |
|--------|--------|
| `/scan` | Lidar 2D (LaserScan) |
| `/odom` | Odometria relativa (GT ou rodas) |
| `/map` | Mapa de ocupação do slam_toolbox |
| TF `map`→`odom`→`base_footprint` | Árvore usada pelo SLAM |
