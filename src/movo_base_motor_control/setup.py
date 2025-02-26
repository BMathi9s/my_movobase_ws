from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'movo_base_motor_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'movo_base_motor_control.nanolib_helper', 
        'movo_base_motor_control.motor_manager',  # 🔹 Ensure motor_manager is included
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all Python scripts in the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Movo Base Maintainer',
    maintainer_email='movo_base@example.com',
    description='Motor control package for omni-wheel base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mecanum_controller = movo_base_motor_control.mecanum_controller:main',
            'omni_wheel_node = movo_base_motor_control.omni_wheel_node:main',
        ],
    },
)
