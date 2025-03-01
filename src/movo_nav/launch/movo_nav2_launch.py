from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ... your other nodes and includes

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('movo_nav2_config'),
                'launch',
                'nav2_bringup.launch.py'
            )
        )
    )

    return LaunchDescription([
        # ... other launch actions,
        nav2_launch
    ])
