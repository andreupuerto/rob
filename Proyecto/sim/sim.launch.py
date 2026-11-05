import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    # Gazebo + TurtleBot3 burger en zona1.world (generado desde el mapa del enunciado)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    world_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'zona1.world')

    tb3_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    actions = [
        IncludeLaunchDescription(   # servidor de Gazebo
            PythonLaunchDescriptionSource(gz_sim),
            launch_arguments={'gz_args': '-r -s -v2 ' + world_file,
                              'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(   # robot + puente ROS <-> Gazebo
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ]
    if gui:
        actions.append(IncludeLaunchDescription(   # ventana de Gazebo
            PythonLaunchDescriptionSource(gz_sim),
            launch_arguments={'gz_args': '-g -v2 '}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.7',
                              description='x de salida (A por defecto)'),
        DeclareLaunchArgument('y_pose', default_value='0.92',
                              description='y de salida (A por defecto)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='true: abre la ventana de Gazebo'),
        OpaqueFunction(function=launch_setup),
    ])
