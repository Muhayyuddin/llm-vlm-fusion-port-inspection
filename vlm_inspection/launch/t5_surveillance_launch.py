#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch T5 surveillance node for quadrotor camera monitoring with AI text analysis
    """
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_topic',
            default_value='/quadrotor_1/slot0/image_raw',
            description='Input image topic from quadrotor camera'
        ),
        
        DeclareLaunchArgument(
            't5_model_name',
            default_value='t5-small',
            description='T5 model for text analysis (t5-small, t5-base, t5-large)'
        ),
        
        DeclareLaunchArgument(
            'vision_model_name',
            default_value='microsoft/Florence-2-base',
            description='Vision model for image descriptions (lighter model for combo with T5)'
        ),
        
        DeclareLaunchArgument(
            'analysis_rate',
            default_value='1.0',
            description='Rate of image analysis in Hz (default: 1 Hz = every 1 second)'
        ),
        
        # T5 Surveillance Node
        Node(
            package='vlm_inspection',
            executable='t5_surveillance_node',
            name='t5_surveillance_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                't5_model_name': LaunchConfiguration('t5_model_name'),
                'vision_model_name': LaunchConfiguration('vision_model_name'),
                'analysis_rate': LaunchConfiguration('analysis_rate'),
            }],
            emulate_tty=True,  # For colored output
        ),
    ])
