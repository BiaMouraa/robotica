"""
Teleoperacao por teclado para explorar o mapa no SLAM.

Rode em um terminal proprio (precisa do foco do teclado):

  ros2 launch meu_robo_description teleop.launch.py

Controles:
  i / ,     frente / tras
  j / l     girar
  k         parar
  q / z     +/- velocidade
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True,
            parameters=[{'stamped': False}],
        ),
    ])
