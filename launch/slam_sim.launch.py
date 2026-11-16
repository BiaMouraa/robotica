"""
Simulacao Gazebo para Pratica 05.2 (SLAM) — sem GPS/EKF (so dead reckoning).

Cenarios (Exercicio 2.2):
  scenario:=gt           Lidar sem ruido + odometria ground truth
  scenario:=lidar_noise  Lidar com ruido + odometria ground truth
  scenario:=both_noise   Lidar com ruido + odometria das rodas com vies

Exemplos:
  ros2 launch meu_robo_description slam_sim.launch.py scenario:=gt
  ros2 launch meu_robo_description slam_sim.launch.py scenario:=lidar_noise
  ros2 launch meu_robo_description slam_sim.launch.py scenario:=both_noise
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _configure(context, *args, **kwargs):
    pkg = get_package_share_directory('meu_robo_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    scenario = LaunchConfiguration('scenario').perform(context)
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world').perform(context)
    if not world:
        world = os.path.join(pkg, 'worlds', 'slam.world')

    # Defaults por cenario
    if scenario == 'gt':
        lidar_noise = '0.0'
        odom_noise = '1.0'
        odom_source = 'groundtruth'
    elif scenario == 'lidar_noise':
        lidar_noise = LaunchConfiguration('lidar_noise_scale').perform(context) or '5.0'
        odom_noise = '1.0'
        odom_source = 'groundtruth'
    elif scenario == 'both_noise':
        lidar_noise = LaunchConfiguration('lidar_noise_scale').perform(context) or '5.0'
        odom_noise = LaunchConfiguration('odom_noise').perform(context) or '1.08'
        odom_source = 'wheels'
    else:
        raise RuntimeError(
            f"scenario invalido: {scenario}. Use gt | lidar_noise | both_noise")

    use_gt = odom_source == 'groundtruth'
    publish_odom_tf = 'false' if use_gt else 'true'
    odometry_topic = 'odom_wheels' if use_gt else 'odom'

    xacro_file = os.path.join(pkg, 'urdf', 'meu_robo.urdf.xacro')
    rviz_config = os.path.join(pkg, 'config', 'slam.rviz')

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            xacro_file, ' ',
            'noise_scale:=1.0 ',
            'lidar_noise_scale:=', lidar_noise, ' ',
            'odom_noise:=', odom_noise, ' ',
            'publish_odom_tf:=', publish_odom_tf, ' ',
            'odometry_topic:=', odometry_topic,
        ]),
        value_type=str,
    )

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'verbose': 'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'meu_robo',
                       '-x', '0.0', '-y', '0.0', '-z', '0.01', '-timeout', '120'],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(rviz),
        ),
    ]

    if use_gt:
        nodes.append(Node(
            package='meu_robo_description',
            executable='gt_odom_node',
            name='gt_odom_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('meu_robo_description')
    default_world = os.path.join(pkg, 'worlds', 'slam.world')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        DeclareLaunchArgument('scenario', default_value='gt',
                              description='gt | lidar_noise | both_noise'),
        DeclareLaunchArgument('lidar_noise_scale', default_value='5.0'),
        DeclareLaunchArgument('odom_noise', default_value='1.08',
                              description='Vies no diametro das rodas (>1 = drift)'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_configure),
    ])
