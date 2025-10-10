#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Comprehensive launch file for all VLM surveillance nodes
    Allows launching multiple surveillance nodes simultaneously for comparison
    """
    
    # Common launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/quadrotor_1/slot0/image_raw',
        description='Input image topic from quadrotor camera'
    )
    
    # Enable/disable specific surveillance nodes
    enable_florence_arg = DeclareLaunchArgument(
        'enable_florence',
        default_value='false',
        description='Enable Florence-2 surveillance node'
    )
    
    enable_blip2_arg = DeclareLaunchArgument(
        'enable_blip2',
        default_value='false',
        description='Enable BLIP2 surveillance node'
    )
    
    enable_nanovlm_arg = DeclareLaunchArgument(
        'enable_nanovlm',
        default_value='true',
        description='Enable nanoVLM surveillance node (~222M parameters)'
    )
    
    enable_smolvlm_arg = DeclareLaunchArgument(
        'enable_smolvlm',
        default_value='true',
        description='Enable SmolVLM surveillance node (256M/500M parameters)'
    )
    
    # Model configuration arguments
    florence_model_arg = DeclareLaunchArgument(
        'florence_model',
        default_value='microsoft/Florence-2-large',
        description='Florence-2 model variant'
    )
    
    blip2_model_arg = DeclareLaunchArgument(
        'blip2_model',
        default_value='Salesforce/blip2-opt-2.7b',
        description='BLIP2 model variant'
    )
    
    nanovlm_model_arg = DeclareLaunchArgument(
        'nanovlm_model',
        default_value='microsoft/nanoVLM',
        description='nanoVLM model variant'
    )
    
    smolvlm_model_arg = DeclareLaunchArgument(
        'smolvlm_model',
        default_value='HuggingFaceTB/SmolVLM-256M-Instruct',
        description='SmolVLM model variant (256M or 500M)'
    )
    
    # Analysis rate arguments (optimized for each model)
    florence_rate_arg = DeclareLaunchArgument(
        'florence_analysis_rate',
        default_value='1.0',
        description='Florence-2 analysis rate in Hz'
    )
    
    blip2_rate_arg = DeclareLaunchArgument(
        'blip2_analysis_rate',
        default_value='0.5',
        description='BLIP2 analysis rate in Hz'
    )
    
    nanovlm_rate_arg = DeclareLaunchArgument(
        'nanovlm_analysis_rate',
        default_value='2.0',
        description='nanoVLM analysis rate in Hz'
    )
    
    smolvlm_rate_arg = DeclareLaunchArgument(
        'smolvlm_analysis_rate',
        default_value='1.5',
        description='SmolVLM analysis rate in Hz'
    )
    
    # Florence-2 Surveillance Node
    florence_surveillance_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_florence')),
        actions=[
            Node(
                package='vlm_inspection',
                executable='surveillance_node',
                name='florence_surveillance_node',
                namespace='florence',
                parameters=[{
                    'input_topic': LaunchConfiguration('input_topic'),
                    'model_name': LaunchConfiguration('florence_model'),
                    'analysis_rate': LaunchConfiguration('florence_analysis_rate')
                }],
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )
    
    # BLIP2 Surveillance Node
    blip2_surveillance_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_blip2')),
        actions=[
            Node(
                package='vlm_inspection',
                executable='blip2_surveillance_node',
                name='blip2_surveillance_node',
                namespace='blip2',
                parameters=[{
                    'input_topic': LaunchConfiguration('input_topic'),
                    'model_name': LaunchConfiguration('blip2_model'),
                    'analysis_rate': LaunchConfiguration('blip2_analysis_rate')
                }],
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )
    
    # nanoVLM Surveillance Node
    nanovlm_surveillance_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_nanovlm')),
        actions=[
            Node(
                package='vlm_inspection',
                executable='nanovlm_surveillance_node',
                name='nanovlm_surveillance_node',
                namespace='nanovlm',
                parameters=[{
                    'input_topic': LaunchConfiguration('input_topic'),
                    'model_name': LaunchConfiguration('nanovlm_model'),
                    'analysis_rate': LaunchConfiguration('nanovlm_analysis_rate'),
                    'enable_vqa': True
                }],
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )
    
    # SmolVLM Surveillance Node
    smolvlm_surveillance_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_smolvlm')),
        actions=[
            Node(
                package='vlm_inspection',
                executable='smolvlm_surveillance_node',
                name='smolvlm_surveillance_node',
                namespace='smolvlm',
                parameters=[{
                    'input_topic': LaunchConfiguration('input_topic'),
                    'model_name': LaunchConfiguration('smolvlm_model'),
                    'analysis_rate': LaunchConfiguration('smolvlm_analysis_rate'),
                    'enable_compliance_check': True,
                    'enable_detailed_vqa': True
                }],
                output='screen',
                emulate_tty=True,
                prefix='stdbuf -o L'
            )
        ]
    )
    
    return LaunchDescription([
        # Common arguments
        input_topic_arg,
        
        # Enable/disable arguments
        enable_florence_arg,
        enable_blip2_arg,
        enable_nanovlm_arg,
        enable_smolvlm_arg,
        
        # Model configuration arguments
        florence_model_arg,
        blip2_model_arg,
        nanovlm_model_arg,
        smolvlm_model_arg,
        
        # Analysis rate arguments
        florence_rate_arg,
        blip2_rate_arg,
        nanovlm_rate_arg,
        smolvlm_rate_arg,
        
        # Surveillance node groups
        florence_surveillance_group,
        blip2_surveillance_group,
        nanovlm_surveillance_group,
        smolvlm_surveillance_group
    ])
