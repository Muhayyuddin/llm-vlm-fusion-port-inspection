#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for nanoVLM surveillance node using ultra-compact VLM (~222M parameters)
    for efficient suspicious activity detection and real-time monitoring
    """
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/quadrotor_1/slot0/image_raw',
        description='Input image topic from quadrotor camera'
    )
    
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='microsoft/Florence-2-base',
        description='Compact VLM model to use for surveillance analysis (using Florence-2-base as efficient alternative)'
    )
    
    analysis_rate_arg = DeclareLaunchArgument(
        'analysis_rate',
        default_value='2.0',
        description='Rate of image analysis in Hz (default: 2 Hz = every 0.5 seconds) - Higher rate due to efficiency'
    )
    
    enable_vqa_arg = DeclareLaunchArgument(
        'enable_vqa',
        default_value='true',
        description='Enable Visual Question Answering for detailed surveillance analysis'
    )
    
    # nanoVLM Surveillance Node
    nanovlm_surveillance_node = Node(
        package='vlm_inspection',
        executable='nanovlm_surveillance_node',
        name='nanovlm_surveillance_node',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_name': LaunchConfiguration('model_name'),
            'analysis_rate': LaunchConfiguration('analysis_rate'),
            'enable_vqa': LaunchConfiguration('enable_vqa')
        }],
        output='screen',
        emulate_tty=True,
        prefix='stdbuf -o L'  # Line buffered output for better real-time display
    )
    
    return LaunchDescription([
        input_topic_arg,
        model_name_arg,
        analysis_rate_arg,
        enable_vqa_arg,
        nanovlm_surveillance_node
    ])
