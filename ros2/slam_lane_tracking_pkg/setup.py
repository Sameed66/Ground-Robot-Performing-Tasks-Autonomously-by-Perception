import os
from glob import glob

from setuptools import setup

package_name = 'slam_lane_tracking_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Optional: install any maps that exist (won't fail if empty)
        (os.path.join('share', package_name, 'maps'),
         glob('slam_lane_tracking_pkg/maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Lane virtual obstacle tools for TurtleBot3',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_logger = slam_lane_tracking_pkg.tf_logger:main',
            'virtual_obstacles_node = slam_lane_tracking_pkg.virtual_obstacles_node:main',
            'centroid_logger = slam_lane_tracking_pkg.centroid_logger_node:main', 
        ],
    },
)