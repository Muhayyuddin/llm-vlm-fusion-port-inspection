#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch BLIP2 surveillance node for quadrotor camera monitoring
    """
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_topic',
            default_value='/quadrotor_1/slot0/image_raw',
            description='Input image topic from quadrotor camera'
        ),
        
        DeclareLaunchArgument(
            'model_name',
            default_value='Salesforce/blip2-opt-2.7b',
            description='BLIP2 model to use for surveillance analysis (memory-optimized default)'
        ),
        
        DeclareLaunchArgument(
            'analysis_rate',
            default_value='1.0',
            description='Rate of image analysis in Hz (default: 1 Hz = every 1 second)'
        ),
        
        # BLIP2 Surveillance Node
        Node(
            package='vlm_inspection',
            executable='blip2_surveillance_node',
            name='blip2_surveillance_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'model_name': LaunchConfiguration('model_name'),
                'analysis_rate': LaunchConfiguration('analysis_rate'),
            }],
            emulate_tty=True,  # For colored output
        ),
    ])
