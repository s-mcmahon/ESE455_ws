from setuptools import setup
import os
from glob import glob

package_name = 'rrt_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # This makes the package visible to ROS
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # This installs package.xml
        ('share/' + package_name, ['package.xml']),
        # This installs all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarah',
    maintainer_email='sarah@example.com',
    description='RRT-based navigation node for TurtleBot3 in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_nav_node = rrt_nav.rrt_nav_node:main',
            'kino_rrt_nav_node = rrt_nav.kino_rrt_nav_node:main',
            'rrt_metrics_node = rrt_nav.rrt_metrics_node:main',
        ],
    },
)
