#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch all surveillance nodes (Florence-2, BLIP2, and T5) for comprehensive monitoring
    """
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'input_topic',
            default_value='/quadrotor_1/slot0/image_raw',
            description='Input image topic from quadrotor camera'
        ),
        
        DeclareLaunchArgument(
            'analysis_rate',
            default_value='0.33',  # Very slow rate for all three models (every 3 seconds)
            description='Rate of image analysis in Hz (0.33 Hz = every 3 seconds for all models)'
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
        
        DeclareLaunchArgument(
            'enable_t5',
            default_value='true',
            description='Enable T5 surveillance node'
        ),
        
        # Model configuration
        DeclareLaunchArgument(
            'florence_model',
            default_value='microsoft/Florence-2-base',  # Lighter model for multi-model setup
            description='Florence-2 model variant'
        ),
        
        DeclareLaunchArgument(
            'blip2_model',
            default_value='Salesforce/blip2-opt-2.7b',
            description='BLIP2 model variant (memory-optimized)'
        ),
        
        DeclareLaunchArgument(
            't5_model',
            default_value='t5-small',
            description='T5 model variant'
        ),
        
        DeclareLaunchArgument(
            't5_vision_model',
            default_value='microsoft/Florence-2-base',
            description='Vision model for T5 surveillance (shared with Florence node)'
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
                    prefix="taskset --cpu-list 0-1",  # Limit CPU usage
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
                    prefix="taskset --cpu-list 2-3",  # Different CPU cores
                ),
            ]
        ),
        
        # T5 Surveillance Node Group
        GroupAction(
            condition=LaunchConfiguration('enable_t5'),
            actions=[
                Node(
                    package='vlm_inspection',
                    executable='t5_surveillance_node',
                    name='t5_surveillance_node',
                    namespace='t5_surveillance',
                    output='screen',
                    parameters=[{
                        'input_topic': LaunchConfiguration('input_topic'),
                        't5_model_name': LaunchConfiguration('t5_model'),
                        'vision_model_name': LaunchConfiguration('t5_vision_model'),
                        'analysis_rate': LaunchConfiguration('analysis_rate'),
                    }],
                    emulate_tty=True,
                    prefix="taskset --cpu-list 4-5",  # Different CPU cores
                ),
            ]
        ),
    ])
