from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='rqt_image_view 실행 여부'
    )

    return LaunchDescription([
        use_rviz_arg,
        # 이미지 퍼블리셔 (camera_pkg)
        Node(package='camera_pkg', executable='img_pub'),
        # YOLO 퍼블리셔 (camera_pkg)
        Node(package='camera_pkg', executable='yolo_pub'),
        # Static TF: map → odom (전역 기준점)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['2.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom']
        ),
        # Dynamic TF: odom → base_link (로봇 주행 시뮬레이션)
        Node(package='tf_tutorial_pkg', executable='odom_sim'),
        # Static TF: base_link → camera_link (센서 마운트)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
        ),
        # Dynamic TF: YOLO 결과를 TF로
        Node(package='tf_tutorial_pkg', executable='tf_yolo'),
        # 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/cchyun/tf_tutorial.rviz'],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])