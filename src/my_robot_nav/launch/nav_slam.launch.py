import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    package_dir = get_package_share_directory('my_robot_nav')

    nav2_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'nav2_slam_params.yaml'),
            description='Full path to Nav2 parameters for SLAM navigation'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 stack'),
        nav2_launch,
    ])
