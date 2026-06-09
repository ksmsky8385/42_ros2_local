import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_map_path = os.path.join(
        get_package_share_directory('my_robot_nav'),
        'maps',
        'map'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value=default_map_path,
            description='Map output path without extension'),
        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'nav2_map_server',
                'map_saver_cli',
                '-f',
                LaunchConfiguration('map_path'),
            ],
            output='screen',
        ),
    ])
