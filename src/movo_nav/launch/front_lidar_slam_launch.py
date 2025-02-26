import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ------------------------------------------------
    # 1. LiDAR node
    # ------------------------------------------------
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim1',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.10',
            'frame_id': 'lidar1_link',
            'tf_base_frame_id': 'odom',
        }],
        remappings=[
            ('sick_tim_5xx/scan', '/lidar_front/scan'),
            ('cloud', '/lidar_front/cloud')
        ]
    )

    # ------------------------------------------------
    # 2. IMU node
    # ------------------------------------------------
 
    imu_node = Node(
        package='tm_imu',
        executable='transducer_m_imu',
        name='tm_imu',
        output='screen',
        # parameters=[ ... ] # if you want to explicitly set param file here
        remappings=[
            ('/imu_data', '/imu_center/data'),
            ('/imu_data_mag', '/imu_center/mag'),
            ('/imu_data_rpy', '/imu_center/rpy'),
        ]
    )

    # ------------------------------------------------
    # 3. (Optional) Static transform between LiDAR and IMU
    # ------------------------------------------------
 
    static_tf_lidar_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_imu_tf',
        arguments=[
            '0', '0', '0.4',      # x y z
            '0', '0', '0', '1', # qx qy qz qw in quaternions
            'lidar1_link',      # parent frame
            'imu_link'          # child frame
        ]
    )
    
                #     A yaw rotation around the Z-axis in ROS can be represented by the quaternion:
                # yaw=15∘=0.261799 radians
                # qx=0,qy=0,qz=sin⁡(yaw/2),qw=cos⁡(yaw/2)
                # qx​=0,qy​=0,qz​=sin(yaw/2),qw​=cos(yaw/2)
   

    # ------------------------------------------------
    # 4. SLAM node
    # ------------------------------------------------
  
    slam_params_file = os.path.join(
    get_package_share_directory('movo_nav'),
    'config',
    'mapper_params_online_sync.yaml'
    )
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )


    return LaunchDescription([
        lidar_node,
        imu_node,
        static_tf_lidar_to_imu,
        slam_node
    ])
