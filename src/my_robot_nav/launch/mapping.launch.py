import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('my_robot_nav'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz on the local PC'),
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Start local joystick manual override'),
        DeclareLaunchArgument(
            'toggle_button',
            default_value='9',
            description='Joystick button index that toggles manual override on/off'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'joy_override.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_joy')),
            launch_arguments={
                'toggle_button': LaunchConfiguration('toggle_button'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
