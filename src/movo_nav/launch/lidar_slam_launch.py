#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory('movo_nav')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    
    # --- Two Lidar Nodes ---
    # Lidar 1 (front) node
    lidar1 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim1',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.10',
            'frame_id': 'lidar1_link',  # using "lidar1_link"
        }],
        remappings=[
            ('scan', '/lidar_front/scan')
        ]
    )

    # Lidar 2 (back) node
    lidar2 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim2',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.11',
            'frame_id': 'lidar2_link',  # using "lidar2_link"
        }],
        remappings=[
            ('scan', '/lidar_back/scan')
        ]
    )

    # --- Static Transforms for Each Lidar ---
    # Position lidar1 relative to base_link (e.g., front sensor offset 0.4 m forward)
    lidar1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar1_tf',
        arguments=['0.4', '0.0', '0.0',  # x, y, z translation (adjust as needed)
                   '0', '0', '0',        # roll, pitch, yaw (radians) - facing forward
                   'base_link', 'lidar1_link']
    )

    # Position lidar2 relative to base_link (e.g., back sensor offset 0.4 m backward, rotated by pi)
    lidar2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar2_tf',
        arguments=['-0.4', '0.0', '0.0',  # x, y, z translation (adjust as needed)
                   '0', '0', '3.14159',   # roll, pitch, yaw - facing opposite direction
                   'base_link', 'lidar2_link']
    )

    # --- (Optional) Static Transform from odom to base_link ---
    # For testing SLAM Toolbox, note that a static odom->base_link will only update once.
    # With a real robot, replace this with a dynamic odometry publisher.
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # --- Dual Lidar Merger Node ---
    # Make sure to mark this file as executable (chmod +x) and add it to your package.
    dual_lidar_merger = Node(
        package='movo_nav',
        executable='dual_lidar_merger',  # This should match the installed executable name.
        name='dual_lidar_merger',
        output='screen'
    )

    # --- SLAM Toolbox Node ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[ParameterFile(slam_params_file, allow_substs=True)],
    )

    # --- (Optional) Robot State Publisher ---
    # If you have a URDF that includes your base and sensor links,
    # you can launch robot_state_publisher to publish the complete TF tree.
    # For a quick test, you can use a minimal URDF.
    minimal_urdf = """
    <robot name="two_lidar_robot">
      <link name="base_link"/>
      <link name="lidar1_link"/>
      <link name="lidar2_link"/>
      <!-- Optionally add joint elements if needed -->
    </robot>
    """
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': minimal_urdf}]
    )

    return LaunchDescription([
        LogInfo(msg=f"Loading SLAM params file: {slam_params_file}"),
        lidar1,
        lidar2,
        lidar1_tf,
        lidar2_tf,
        odom_tf,
        rsp_node,
        slam_toolbox_node
    ])
