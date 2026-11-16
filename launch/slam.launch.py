"""
SLAM 2D com slam_toolbox (Pratica 05.2).

Requer slam_sim.launch.py ja em execucao (ou o mesmo ambiente).

mode:=mapping       constroi o mapa (Exercicios 2.1 / 2.2)
mode:=localization  localiza no mapa salvo (Exercicio 3.1 - robo sequestrado)

Salvar mapa (apos explorar):
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
    "{name: {data: '/tmp/meu_robo_slam_map'}}"

Exemplos:
  ros2 launch meu_robo_description slam.launch.py
  ros2 launch meu_robo_description slam.launch.py mode:=localization
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configure(context, *args, **kwargs):
    pkg = get_package_share_directory('meu_robo_description')
    mode = LaunchConfiguration('mode').perform(context)

    if mode == 'localization':
        params = os.path.join(pkg, 'config', 'mapper_params_localization.yaml')
        executable = 'localization_slam_toolbox_node'
    else:
        params = os.path.join(pkg, 'config', 'mapper_params_online_async.yaml')
        executable = 'async_slam_toolbox_node'

    map_file = LaunchConfiguration('map_file').perform(context)
    extra = {'use_sim_time': True}
    if mode == 'localization' and map_file:
        extra['map_file_name'] = map_file

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[params, extra],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='mapping | localization'),
        DeclareLaunchArgument(
            'map_file',
            default_value='/tmp/meu_robo_slam_map',
            description='Prefixo do pose-graph salvo pelo slam_toolbox'),
        OpaqueFunction(function=_configure),
    ])
