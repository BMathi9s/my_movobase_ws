from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # set default accel and decek
    set_acceleration = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/movo_base/set_acceleration',
            'std_msgs/msg/Float32MultiArray',
            '{data: [0.5]}'
        ],
        output='screen'
    )

    set_deceleration = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/movo_base/set_deceleration',
            'std_msgs/msg/Float32MultiArray',
            '{data: [10.0]}'
        ],
        output='screen'
    )
    # set default initial speed to 0 : ros2 topic pub /movo_base/wheel_cmds std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

    set_init_velocity = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/movo_base/wheel_cmds',
            'std_msgs/msg/Float32MultiArray',
            '{data: [0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )

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
        #  Launch forward_kinematic node
        # Node(
        #     package='movo_base_motor_control',
        #     executable='forward_kinematic',
        #     name='forward_kinematic_node',
        #     output='screen'
        # ),
        # Publish acceleration command once
        set_acceleration,
        # Publish deceleration command once
        set_deceleration,
        set_init_velocity,
    ])
