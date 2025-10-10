#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Bridge the LiDAR sensor
    lidar_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/coast-port/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
        ],
        remappings=[
            ('/world/coast-port/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan', '/usv/slot0/scan')
        ],
        output='screen'
    )

    return LaunchDescription([
        lidar_bridge
    ])
