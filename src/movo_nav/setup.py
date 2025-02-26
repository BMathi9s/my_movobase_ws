from setuptools import setup
from glob import glob
import os

package_name = 'movo_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS needs to know this is a ROS package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # The package.xml
        ('share/' + package_name, ['package.xml']),
        # Install our launch files
        ('share/' + package_name + '/launch', ['launch/full_2d_lidar_slam_launch.py']),
        ('share/' + package_name + '/launch', ['launch/single_2d_lidar_slam_with_urdf_launch.py']),
        ('share/' + package_name + '/launch', ['launch/front_lidar_slam_launch.py']),
        ('share/' + package_name + '/launch', ['launch/lidar_slam_launch.py']),
        
        # -------------- Add these lines --------------
        # Install anything in the urdf folder
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')
        ),
        # If you have other URDF files or subfolders, do likewise

        # Install anything in the config folder
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        # ---------------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='movo_base',
    maintainer_email='movo@example.com',
    description='Movo navigation stack',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Any Python entry points go here
            'dual_scan_merger = movo_nav.dual_scan_merger:main'
        ],
    },
)
