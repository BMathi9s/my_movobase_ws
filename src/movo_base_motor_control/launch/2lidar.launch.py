# two_tims.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    tim1 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim1',  # node name
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.10',
            'frame_id': 'lidar1_link',
            # 'port': 2112,  # if needed
        }],
        remappings=[
            ('cloud', '/lidar_front/cloud'),
            ('sick_tim_5xx/scan', '/lidar_front/scan') 
        ]
    )

    tim2 = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim2',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.11',
            'frame_id': 'lidar2_link',
            # 'port': 2112,  # if needed
        }],
        remappings=[
            ('cloud', '/lidar_back/cloud'),
            ('sick_tim_5xx/scan', '/lidar_back/scan')  # remap the LaserScan topic
        ]
    )

    return LaunchDescription([tim1, tim2])
