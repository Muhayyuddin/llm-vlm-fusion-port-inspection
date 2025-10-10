from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uav_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='UAV navigation package with PID control and waypoint following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_navigation_node = uav_navigation.uav_navigation_node:main',
            'uav_odom_tf_broadcaster = uav_navigation.uav_odom_tf_broadcaster:main',
        ],
    },
)