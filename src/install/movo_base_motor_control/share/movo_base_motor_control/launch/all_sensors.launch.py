#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Node for the first Sick LiDAR
    tim1 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim1',  # node name
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.10',
            'frame_id': 'lidar1_link',
        }],
        remappings=[
            ('cloud', '/lidar_front/cloud'),
            ('scan', '/lidar_front/scan')
        ]
    )

    # Node for the second Sick LiDAR
    tim2 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim2',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.11',
            'frame_id': 'lidar2_link',
        }],
        remappings=[
            ('cloud', '/lidar_back/cloud'),
            ('scan', '/lidar_back/scan')
        ]
    )

    # Include the Intel RealSense launch file.
    # This will start the RealSense camera node and any associated nodes.
    realsense_pkg_share = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        # If you need to pass any launch arguments, you can do so like this:
        launch_arguments={'filters': 'pointcloud'}.items()
    )

    # Return all nodes as part of the launch description.
    return LaunchDescription([
        tim1,
        tim2,
        realsense_camera
    ])
