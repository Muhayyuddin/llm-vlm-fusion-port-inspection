#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for surveillance node using Florence-2 for suspicious activity detection
    """
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/quadrotor_1/slot0/image_raw',
        description='Input image topic from quadrotor camera'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='microsoft/Florence-2-large',
        description='Florence-2 model to use for surveillance analysis'
    )
    
    analysis_rate_arg = DeclareLaunchArgument(
        'analysis_rate',
        default_value='1.0',
        description='Rate of image analysis in Hz (default: 1 Hz = every 1 second)'
    )
    
    # Surveillance Node
    surveillance_node = Node(
        package='vlm_inspection',
        executable='surveillance_node',
        name='surveillance_node',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_name': LaunchConfiguration('model_name'),
            'analysis_rate': LaunchConfiguration('analysis_rate')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        model_name_arg,
        analysis_rate_arg,
        surveillance_node
    ])
