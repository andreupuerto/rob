import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    goal_x = float(LaunchConfiguration('goal_x').perform(context))
    goal_y = float(LaunchConfiguration('goal_y').perform(context))
    advanced = LaunchConfiguration('advanced').perform(context).lower() == 'true'

    # Nodo Bug2
    actions = [
        Node(
            package='package_bug2_11j_basic',
            executable='bug2',
            name='bug2_node',
            output='screen',
            parameters=[{'use_sim_time': sim,   # Essential for Gazebo
                         'goal_x': goal_x,
                         'goal_y': goal_y,
                         'advanced': advanced}],
        ),
    ]

    if sim:
        # Gazebo + TurtleBot3 burger en el mundo elegido (igual que empty_world.launch.py
        # de turtlebot3_gazebo, pero cargando uno de nuestros mundos)
        tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
        tb3_launch = os.path.join(tb3_gazebo, 'launch')
        gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        if world == 'empty':
            world_file = os.path.join(tb3_gazebo, 'worlds', 'empty_world.world')
        else:
            world_file = os.path.join(
                get_package_share_directory('package_bug2_11j_basic'), 'worlds', world + '.world')

        actions += [
            IncludeLaunchDescription(   # servidor de Gazebo
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={'gz_args': '-r -s -v2 ' + world_file,
                                  'on_exit_shutdown': 'true'}.items()),
            IncludeLaunchDescription(   # ventana de Gazebo
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={'gz_args': '-g -v2 '}.items()),
            IncludeLaunchDescription(   # robot en (0, 0) + puente ROS <-> Gazebo
                PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
                launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='true: arranca Gazebo. false: robot real'),
        DeclareLaunchArgument('world', default_value='box',
                              description='Mundo de Gazebo: empty, box, u_shape, multi o enclosed'),
        DeclareLaunchArgument('goal_x', default_value='2.7',
                              description='Objetivo X (m, frame odom, relativo a la pose inicial)'),
        DeclareLaunchArgument('goal_y', default_value='0.0',
                              description='Objetivo Y (m, frame odom, relativo a la pose inicial)'),
        DeclareLaunchArgument('advanced', default_value='false',
                              description='true: sale de la pared en cuanto hay camino libre al objetivo'),
        OpaqueFunction(function=launch_setup),
    ])
