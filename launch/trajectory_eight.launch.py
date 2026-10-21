"""
Inicia o seguidor de trajetoria em 8 (Exercicio 2.1).

Argumentos:
  use_sensors : true  -> pose estimada por IMU (yaw) + GPS (x,y)  [padrao]
                false -> pose vinda de /odom (pratica anterior)
  params_file : arquivo YAML de parametros (padrao config/trajectory_eight.yaml)

Exemplos:
  ros2 launch meu_robo_description trajectory_eight.launch.py
  ros2 launch meu_robo_description trajectory_eight.launch.py use_sensors:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('meu_robo_description')
    default_params = os.path.join(pkg, 'config', 'trajectory_eight.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sensors', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='meu_robo_description',
            executable='trajectory_eight_node',
            name='trajectory_eight_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': True,
                    'use_sensors': ParameterValue(LaunchConfiguration('use_sensors'), value_type=bool),
                },
            ],
        ),
    ])
