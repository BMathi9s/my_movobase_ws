from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Launch joy_to_twist_node
        Node(
            package='joy_to_twist',
            executable='joy_to_twist_node',
            name='joy_to_twist_node',
            output='screen'
        ),

        # Launch omni_wheel_node
        Node(
            package='movo_base_motor_control',
            executable='omni_wheel_node',
            name='omni_wheel_node',
            output='screen'
        ),

        # Launch mecanum_controller
        Node(
            package='movo_base_motor_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen'
        ),
    ])
