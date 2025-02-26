import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    movo_description_share = get_package_share_directory('movo_description')
    movo_nav_share = get_package_share_directory('movo_nav')
    
    # -----------------------------------------------------------
    # 1. Include the robot display (mod_display.launch.py)
    # -----------------------------------------------------------
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(movo_description_share, 'launch', 'mod_display.launch.py')
        )
    )
    
    # -----------------------------------------------------------
    # 2. Front Lidar Node (using front_laser_link)
    # -----------------------------------------------------------
    front_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim1',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.10',
            'frame_id': 'front_laser_link',
            'tf_base_frame_id': 'front_laser_link',
            'tf_publish_rate': 0.0,
              
            # # Set the scan angle range:
            # 'min_ang': 0.785398163,   # 45 degrees in radians
            # 'max_ang': 4.27605667,    # 245 degrees in radians
            # # Set the range (distance) limits:
            # 'range_min': 0.1,         # Minimum valid range (in meters)
            # 'range_max': 30.0,        # Maximum valid range (in meters)
            # # Additional optional parameters:
            # 'intensity': True,
            # # You can add other parameters here as needed:
            # # 'use_binary_protocol': True,
            # # 'range_filter_handling': 0,
            # # etc.
        }],
        remappings=[
            ('sick_tim_5xx/scan', '/lidar_front/scan'),
            ('cloud', '/lidar_front/cloud')
        ]
    )
    

    
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    # -----------------------------------------------------------
    # 3. SLAM Toolbox Node with Dual Lidar Parameters
    # -----------------------------------------------------------
    slam_params_file = os.path.join(movo_nav_share, 'config', 'dual_lidar_slam_param.yaml')
    slam_node = TimerAction(
        period=2.0,  # delay of 2 seconds o fix the initil timing issue
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params_file]
            )
        ]
    )

    
    return LaunchDescription([
        display_launch,
        front_lidar_node,
        odom_tf,
        slam_node
    ])
