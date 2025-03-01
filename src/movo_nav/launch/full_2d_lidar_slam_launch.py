import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # -----------------------------------------------------------
    # 3. Rear Lidar Node (using rear_laser_link)
    # -----------------------------------------------------------
    back_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim2',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.11',
            'frame_id': 'rear_laser_link',
            'tf_base_frame_id': 'rear_laser_link',
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
            ('sick_tim_5xx/scan', '/lidar_back/scan'),
            ('cloud', '/lidar_back/cloud')
        ]
    )
    
    # -----------------------------------------------------------
    # 4. IMU Node
    # -----------------------------------------------------------
    imu_node = Node(
        package='tm_imu',
        executable='transducer_m_imu',
        name='tm_imu',
        output='screen',
        remappings=[
            ('/imu_data', '/imu_center/data'),
            ('/imu_data_mag', '/imu_center/mag'),
            ('/imu_data_rpy', '/imu_center/rpy'),
        ]
    )
    
    # -----------------------------------------------------------
    # 5. Static Transform from Front Lidar to IMU
    # -----------------------------------------------------------
    static_tf_front_lidar_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_lidar_to_imu_tf',
        arguments=[
            '0', '0', '0.4',      # translation: x, y, z
            '0', '0', '0', '1',    # rotation as quaternion: (qx, qy, qz, qw)
            'front_laser_link',    # parent frame
            'imu_link'             # child frame
        ]
    )
    
      # -----------------------------------------------------------
    # 6. Dual Scan Merger Node (your custom merger)
    # -----------------------------------------------------------
    dual_lidar_merger = Node(
        package='movo_nav',
        executable='dual_scan_merger',  # entry point in your package
        name='dual_lidar_merger',
        output='screen',
        parameters=[{
            'output_frame': 'base_link',
            'scan_topics': ['/lidar_front/scan', '/lidar_back/scan'],
            'merged_scan_topic': '/merged_scan'
        }]
    )
    
    # -----------------------------------------------------------
    # 7. SLAM Toolbox Node with Dual Lidar Parameters
    # -----------------------------------------------------------
    slam_params_file = os.path.join(movo_nav_share, 'config', 'dual_lidar_slam_param.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )
    
    return LaunchDescription([
        display_launch,
        front_lidar_node,
        back_lidar_node,
        # imu_node,
        # static_tf_front_lidar_to_imu,
        # dual_lidar_merger,
        slam_node
    ])
