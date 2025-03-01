import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                get_package_share_directory('movo_nav2_config'),
                'config',
                'nav2_params.yaml'
            ),
            # Optionally, if you have a map file:
            'map': os.path.join(
                get_package_share_directory('movo_nav2_config'),
                'maps',
                'your_map.yaml'
            )
        }.items()
    )

    return LaunchDescription([nav2_launch])
