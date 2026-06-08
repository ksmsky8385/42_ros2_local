import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'one_dof_arm.xacro')

    declare_payload = DeclareLaunchArgument(
        'payload_mass',
        default_value='1.0',
        description='Payload mass in kg (default: 1.0kg)'
    )

    robot_description = Command([
        'xacro ', xacro_file,
        ' payload_mass:=', LaunchConfiguration('payload_mass')
    ])

    # Gazebo Classic 실행
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Gazebo에 로봇 스폰
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'one_dof_arm',
            '-z', '1.0',
        ],
        output='screen',
    )

    # 컨트롤러 로드 (스폰 완료 후 순차 실행)
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    load_effort = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '-c', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        declare_payload,
        gazebo,
        rsp,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_effort],
            )
        ),
    ])
