import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    # 런치 인자 선언
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='rqt_image_view 실행 여부'
    )

    config = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params.yaml'
    )
        
    # image_publisher
    image_publisher = Node(
        package='camera_pkg',
        executable='img_pub',
        parameters=[config]
    )

    # yolo_publisher
    yolo_publisher = Node(
        package='camera_pkg',
        executable='yolo_pub',
        parameters=[config]
    )

    # rqt_image_view (조건부 실행)
    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        image_publisher,
        yolo_publisher,
        viewer_node,
    ])

