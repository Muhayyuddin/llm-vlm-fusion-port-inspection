from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'unified_mission_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhayy',
    maintainer_email='muhayy@example.com',
    description='Unified Mission Planner for USV and UAV Heterogeneous System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_mission_planner = unified_mission_planner.unified_mission_planner:main',
            'mission_coordinator = unified_mission_planner.mission_coordinator:main',
            'mission_test = unified_mission_planner.mission_test:main',
            'heterogeneous_mission_planner = unified_mission_planner.heterogeneous_mission_planner:main',
        ],
    },
)
