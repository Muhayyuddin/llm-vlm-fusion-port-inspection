#!/usr/bin/env python3
"""
Launch file for PaliGemma Inspection System

This launch file deploys Google's PaliGemma vision-language model for
comprehensive image inspection and analysis tasks.

PaliGemma Features:
- SigLIP vision encoder + Gemma language model
- High-quality visual question answering
- Detailed image captioning and description
- Multi-task inspection analysis
- Surveillance and safety monitoring

Author: VLM Inspection Team
Date: 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for PaliGemma inspection system
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # Declare launch arguments
    inspection_type_arg = DeclareLaunchArgument(
        'inspection_type',
        default_value='general',
        description='Type of inspection: general, safety, quality, maintenance, infrastructure'
    )
    
    inspection_frequency_arg = DeclareLaunchArgument(
        'inspection_frequency',
        default_value='1.0',
        description='Image processing frequency in Hz (recommended: 0.5-2.0 for PaliGemma)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='ROS2 logging level: DEBUG, INFO, WARN, ERROR'
    )
    
    enable_vqa_arg = DeclareLaunchArgument(
        'enable_vqa',
        default_value='true',
        description='Enable detailed visual question answering'
    )
    
    enable_surveillance_arg = DeclareLaunchArgument(
        'enable_surveillance',
        default_value='true',
        description='Enable surveillance and security analysis'
    )
    
    model_variant_arg = DeclareLaunchArgument(
        'model_variant',
        default_value='microsoft/Florence-2-base-ft',
        description='PaliGemma-style model variant to use (using Florence-2 as alternative)'
    )
    
    # PaliGemma Service Node - AI Backend
    paligamma_service_node = Node(
        package='vlm_inspection',
        executable='paligamma_service',
        name='paligamma_service_node',
        namespace='vlm_inspection',
        output='screen',
        parameters=[{
            'model_optimization': True,
            'enable_gradient_checkpointing': True,
            'low_cpu_mem_usage': True,
            'torch_dtype': 'float16',
            'max_new_tokens': 256,
            'temperature': 0.1,
            'do_sample': False
        }],
        remappings=[
            ('paligamma_analysis', '/vlm_inspection/paligamma_analysis'),
            ('paligamma_vqa', '/vlm_inspection/paligamma_vqa'),
            ('paligamma_caption', '/vlm_inspection/paligamma_caption'),
            ('paligamma_inspection', '/vlm_inspection/paligamma_inspection')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
        # PaliGemma Inspection Node - Advanced Analysis
    paligamma_inspection_node = Node(
        package='vlm_inspection',
        executable='paligamma_inspection_node',
        name='paligamma_inspection_analyzer',
        namespace='vlm_inspection',
        output='screen',
        parameters=[{
            'input_topic': '/quadrotor_1/slot0/image_raw',
            'inspection_frequency': LaunchConfiguration('inspection_frequency'),
            'inspection_type': LaunchConfiguration('inspection_type'),
            'enable_vqa': LaunchConfiguration('enable_vqa'),
            'enable_surveillance': LaunchConfiguration('enable_surveillance'),
            'model_name': LaunchConfiguration('model_variant'),
            'batch_size': 1,
            'max_new_tokens': 256,
            'temperature': 0.7,
            'top_p': 0.9,
            'confidence_threshold': 0.7
        }],
        remappings=[
            ('paligamma_inspection_result', '/vlm_inspection/paligamma_inspection_result'),
            ('paligamma_caption', '/vlm_inspection/paligamma_caption'),
            ('paligamma_vqa_result', '/vlm_inspection/paligamma_vqa_result'),
            ('paligamma_surveillance_alert', '/vlm_inspection/paligamma_surveillance_alert')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # System initialization info
    system_info = LogInfo(
        msg='\n========================================\n'
            '🤖 PALIGAMMA INSPECTION SYSTEM LAUNCHED\n'
            '========================================\n'
            '🔍 Model: Google PaliGemma (SigLIP + Gemma)\n'
            '🎯 Capabilities: Vision-Language Understanding\n'
            '❓ VQA: High-quality visual question answering\n'
            '📝 Captioning: Detailed image descriptions\n'
            '🔧 Inspection: Multi-task analysis\n'
            '🚨 Surveillance: Security monitoring\n'
            '🧠 Architecture: SigLIP vision + Gemma language\n'
            '========================================\n'
            '📡 Subscribed Topics:\n'
            '   • /camera/image_raw (sensor_msgs/Image)\n'
            '📢 Published Topics:\n'
            '   • /vlm_inspection/paligamma_inspection_result (std_msgs/String)\n'
            '   • /vlm_inspection/paligamma_caption (std_msgs/String)\n'
            '   • /vlm_inspection/paligamma_vqa_result (std_msgs/String)\n'
            '   • /vlm_inspection/paligamma_surveillance_alert (std_msgs/String)\n'
            '🛠️  Available Services:\n'
            '   • /vlm_inspection/paligamma_analysis\n'
            '   • /vlm_inspection/paligamma_vqa\n'
            '   • /vlm_inspection/paligamma_caption\n'
            '   • /vlm_inspection/paligamma_inspection\n'
            '========================================\n'
            '🚀 PaliGemma Inspection System Ready!\n'
            '   Processing images with Google\'s vision-language model...\n'
            '========================================'
    )
    
    # Launch description with all components
    return LaunchDescription([
        # Launch arguments
        inspection_type_arg,
        inspection_frequency_arg,
        log_level_arg,
        enable_vqa_arg,
        enable_surveillance_arg,
        model_variant_arg,
        
        # System information
        system_info,
        
        # Core nodes
        paligamma_service_node,
        paligamma_inspection_node
    ])


if __name__ == '__main__':
    generate_launch_description()
