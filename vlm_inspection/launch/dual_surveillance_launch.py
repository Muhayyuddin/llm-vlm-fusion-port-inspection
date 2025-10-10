#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch both Florence-2 and BLIP2 surveillance nodes for comprehensive monitoring
    """
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_topic',
            default_value='/quadrotor_1/slot0/image_raw',
            description='Input image topic from quadrotor camera'
        ),
        
        DeclareLaunchArgument(
            'florence_model',
            default_value='microsoft/Florence-2-large',
            description='Florence-2 model to use for surveillance analysis'
        ),
        
        DeclareLaunchArgument(
            'blip2_model',
            default_value='Salesforce/blip2-opt-2.7b',
            description='BLIP2 model to use for surveillance analysis (memory-optimized)'
        ),
        
        DeclareLaunchArgument(
            'analysis_rate',
            default_value='0.5',  # Slower rate for dual models to avoid memory issues
            description='Rate of image analysis in Hz (0.5 Hz = every 2 seconds for dual analysis)'
        ),
        
        DeclareLaunchArgument(
            'enable_florence',
            default_value='true',
            description='Enable Florence-2 surveillance node'
        ),
        
        DeclareLaunchArgument(
            'enable_blip2',
            default_value='true',
            description='Enable BLIP2 surveillance node'
        ),
        
        # Florence-2 Surveillance Node Group
        GroupAction(
            condition=LaunchConfiguration('enable_florence'),
            actions=[
                Node(
                    package='vlm_inspection',
                    executable='surveillance_node',
                    name='florence_surveillance_node',
                    namespace='florence_surveillance',
                    output='screen',
                    parameters=[{
                        'input_topic': LaunchConfiguration('input_topic'),
                        'model_name': LaunchConfiguration('florence_model'),
                        'analysis_rate': LaunchConfiguration('analysis_rate'),
                    }],
                    emulate_tty=True,
                ),
            ]
        ),
        
        # BLIP2 Surveillance Node Group  
        GroupAction(
            condition=LaunchConfiguration('enable_blip2'),
            actions=[
                Node(
                    package='vlm_inspection',
                    executable='blip2_surveillance_node',
                    name='blip2_surveillance_node',
                    namespace='blip2_surveillance',
                    output='screen',
                    parameters=[{
                        'input_topic': LaunchConfiguration('input_topic'),
                        'model_name': LaunchConfiguration('blip2_model'),
                        'analysis_rate': LaunchConfiguration('analysis_rate'),
                    }],
                    emulate_tty=True,
                ),
            ]
        ),
    ])
