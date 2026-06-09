from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    toggle_button = LaunchConfiguration('toggle_button')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='local_joy_node',
        output='screen',
    )

    manual_override_node = Node(
        package='my_robot_nav',
        executable='manual_override',
        name='manual_override',
        output='screen',
        parameters=[{
            'toggle_button': ParameterValue(toggle_button, value_type=int),
            'axis_linear_x': 1,
            'axis_angular_yaw': 0,
            'scale_linear_x': 0.20,
            'scale_angular_yaw': 2.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'toggle_button',
            default_value='9',
            description='Joystick button index that toggles manual override on/off'),
        joy_node,
        manual_override_node,
    ])
