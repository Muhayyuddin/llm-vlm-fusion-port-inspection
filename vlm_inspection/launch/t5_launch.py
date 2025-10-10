#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch T5 node for text processing
    """
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'model_name',
            default_value='t5-small',
            description='T5 model to use (t5-small, t5-base, t5-large)'
        ),
        
        DeclareLaunchArgument(
            'use_vision_model',
            default_value='true',
            description='Use vision model for image processing'
        ),
        
        DeclareLaunchArgument(
            'vision_model_name',
            default_value='microsoft/Florence-2-base',
            description='Vision model to use with T5 for image descriptions'
        ),
        
        # T5 Node
        Node(
            package='vlm_inspection',
            executable='t5_node',
            name='t5_node',
            output='screen',
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'use_vision_model': LaunchConfiguration('use_vision_model'),
                'vision_model_name': LaunchConfiguration('vision_model_name'),
            }],
            emulate_tty=True,  # For colored output
        ),
    ])
