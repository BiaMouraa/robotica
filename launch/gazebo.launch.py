"""
Simulacao do robo no Gazebo Classic com os sensores da Pratica 04.

Argumentos:
  noise_scale  : multiplicador do ruido de TODOS os sensores (padrao 1.0)
  world        : arquivo .world (padrao: worlds/sensores.world)
  rviz         : abre o RViz2 com config de sensores (padrao false)
  plotjuggler  : abre o PlotJuggler (padrao false)
  gui          : abre o cliente grafico do Gazebo (padrao true)

Exemplos:
  ros2 launch meu_robo_description gazebo.launch.py
  ros2 launch meu_robo_description gazebo.launch.py rviz:=true plotjuggler:=true
  ros2 launch meu_robo_description gazebo.launch.py noise_scale:=5.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_meu_robo = get_package_share_directory('meu_robo_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_meu_robo, 'urdf', 'meu_robo.urdf.xacro')
    default_world = os.path.join(pkg_meu_robo, 'worlds', 'sensores.world')
    rviz_config = os.path.join(pkg_meu_robo, 'config', 'sensores.rviz')
    plotjuggler_layout = os.path.join(pkg_meu_robo, 'config', 'sensores.plotjuggler.xml')

    noise_scale = LaunchConfiguration('noise_scale')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_plotjuggler = LaunchConfiguration('plotjuggler')
    gui = LaunchConfiguration('gui')

    declare_args = [
        DeclareLaunchArgument('noise_scale', default_value='1.0',
                              description='Multiplicador do ruido dos sensores'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Arquivo .world do Gazebo'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir RViz2'),
        DeclareLaunchArgument('plotjuggler', default_value='false',
                              description='Abrir PlotJuggler'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir gzclient'),
    ]

    # xacro processado em tempo de launch para aceitar noise_scale
    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            xacro_file, ' ',
            'noise_scale:=', noise_scale,
        ]),
        value_type=str,
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'meu_robo',
                   '-z', '0.01', '-timeout', '120'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # PlotJuggler ja com layout (IMU, GPS, trajetoria em 8) e streaming ROS 2 ativo
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        # Nome do plugin (nao o .so). Sem isso o PlotJuggler abre sem
        # streaming e os graficos ficam estaticos.
        arguments=['--nosplash', '--layout', plotjuggler_layout,
                   '--start_streamer', 'ROS2 Topic Subscriber',
                   '--buffer_size', '120'],
        output='screen',
        condition=IfCondition(use_plotjuggler),
    )

    return LaunchDescription(declare_args + [
        # Evita o Gazebo tentar consultar o banco de modelos online na
        # inicializacao (demora/trava sem internet; os modelos sao locais).
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        node_robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        rviz,
        plotjuggler,
    ])
