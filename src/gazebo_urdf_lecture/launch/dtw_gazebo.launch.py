import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('gazebo_urdf_lecture')
    xacro_file = os.path.join(pkg, 'xacro', 'dtw_robot.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}],
               output='screen')

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'dtw_robot'],
                 output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo, rsp, spawn,
    ])
