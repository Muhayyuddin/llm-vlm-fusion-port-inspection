#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for SmolVLM surveillance node using minimal footprint VLM (256M/500M parameters)
    with strong multimodal capability for advanced surveillance and compliance monitoring
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
        description='SmolVLM-style model variant (using Florence-2-large for advanced capabilities)'
    )
    
    analysis_rate_arg = DeclareLaunchArgument(
        'analysis_rate',
        default_value='1.5',
        description='Rate of image analysis in Hz (default: 1.5 Hz = every 0.67 seconds)'
    )
    
    enable_compliance_check_arg = DeclareLaunchArgument(
        'enable_compliance_check',
        default_value='true',
        description='Enable regulatory compliance assessment for maritime safety'
    )
    
    enable_detailed_vqa_arg = DeclareLaunchArgument(
        'enable_detailed_vqa',
        default_value='true',
        description='Enable detailed Visual Question Answering for comprehensive scene analysis'
    )
    
    # SmolVLM Surveillance Node
    smolvlm_surveillance_node = Node(
        package='vlm_inspection',
        executable='smolvlm_surveillance_node',
        name='smolvlm_surveillance_node',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_name': LaunchConfiguration('model_name'),
            'analysis_rate': LaunchConfiguration('analysis_rate'),
            'enable_compliance_check': LaunchConfiguration('enable_compliance_check'),
            'enable_detailed_vqa': LaunchConfiguration('enable_detailed_vqa')
        }],
        output='screen',
        emulate_tty=True,
        prefix='stdbuf -o L'  # Line buffered output for better real-time display
    )
    
    return LaunchDescription([
        input_topic_arg,
        model_name_arg,
        analysis_rate_arg,
        enable_compliance_check_arg,
        enable_detailed_vqa_arg,
        smolvlm_surveillance_node
    ])
