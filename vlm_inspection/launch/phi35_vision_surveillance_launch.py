#!/usr/bin/env python3
"""
Launch file for Phi-3.5-vision-instruct Comprehensive Surveillance System

This launch file deploys the most advanced VLM surveillance service and node
for comprehensive multi-modal scene understanding, threat assessment, and 
regulatory compliance monitoring.

Phi-3.5-vision-instruct Features:
- State-of-the-art multi-frame image understanding
- Advanced visual reasoning and question answering
- Document and chart comprehension
- Superior threat detection with 4.2B parameters
- Multi-image comparison and analysis
- Detailed security recommendations

Author: VLM Inspection Team
Date: 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for Phi-3.5-vision-instruct comprehensive surveillance system
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # Declare launch arguments for advanced configuration
    threat_sensitivity_arg = DeclareLaunchArgument(
        'threat_sensitivity',
        default_value='medium',
        description='Threat detection sensitivity level: low, medium, high'
    )
    
    surveillance_frequency_arg = DeclareLaunchArgument(
        'surveillance_frequency',
        default_value='2.0',
        description='Image processing frequency in Hz (recommended: 1.0-3.0 for Phi-3.5 Vision)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='ROS2 logging level: DEBUG, INFO, WARN, ERROR'
    )
    
    use_advanced_analysis_arg = DeclareLaunchArgument(
        'use_advanced_analysis',
        default_value='true',
        description='Enable advanced multi-task scene understanding'
    )
    
    compliance_monitoring_arg = DeclareLaunchArgument(
        'compliance_monitoring',
        default_value='true',
        description='Enable regulatory compliance monitoring'
    )
    
    # Phi-3.5-vision-instruct Service Node - Advanced AI Backend
    phi35_vision_service_node = Node(
        package='vlm_inspection',
        executable='phi35_vision_service',
        name='phi35_vision_service_node',
        namespace='vlm_surveillance',
        output='screen',
        parameters=[{
            'model_optimization': True,
            'enable_scene_understanding': True,
            'enable_advanced_vqa': True,
            'enable_compliance_check': True,
            'memory_optimization': True,
            'gradient_checkpointing': True,
            'mixed_precision': True,
            'batch_size': 1,
            'max_new_tokens': 512,
            'temperature': 0.1,
            'top_p': 0.9,
            'repetition_penalty': 1.1
        }],
        remappings=[
            ('phi35_vision_analysis', '/vlm_surveillance/phi35_vision_analysis'),
            ('scene_understanding', '/vlm_surveillance/scene_understanding'),
            ('advanced_vqa', '/vlm_surveillance/advanced_vqa'),
            ('compliance_assessment', '/vlm_surveillance/compliance_assessment')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Phi-3.5 Vision Comprehensive Surveillance Node - Advanced Monitoring
    phi35_vision_surveillance_node = Node(
        package='vlm_inspection',
        executable='phi35_vision_surveillance_node',
        name='phi35_vision_comprehensive_surveillance',
        namespace='vlm_surveillance',
        output='screen',
        parameters=[{
            'threat_sensitivity': LaunchConfiguration('threat_sensitivity'),
            'surveillance_frequency': LaunchConfiguration('surveillance_frequency'),
            'enable_multi_task_analysis': LaunchConfiguration('use_advanced_analysis'),
            'enable_compliance_monitoring': LaunchConfiguration('compliance_monitoring'),
            'enable_advanced_vqa': True,
            'enable_detailed_reporting': True,
            'confidence_threshold': 0.7,
            'max_detections': 10,
            'scene_understanding_tasks': [
                '<OD>',  # Object Detection
                '<DENSE_REGION_CAPTION>',  # Dense Region Captioning
                '<REGION_PROPOSAL>',  # Region Proposal
                '<CAPTION_TO_PHRASE_GROUNDING>',  # Caption to Phrase Grounding
                '<OCR_WITH_REGION>'  # OCR with Region
            ],
            'advanced_vqa_questions': [
                'What security concerns are visible in this scene?',
                'Are there any safety violations or hazards present?',
                'What type of environment or facility is this?',
                'Are there any unauthorized personnel or activities?',
                'What equipment or objects require security attention?'
            ],
            'compliance_standards': [
                'industrial_safety',
                'facility_security',
                'access_control',
                'equipment_protocols',
                'emergency_procedures'
            ]
        }],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('surveillance_alert', '/vlm_surveillance/phi35_vision_alert'),
            ('threat_assessment', '/vlm_surveillance/phi35_vision_threat_assessment'),
            ('compliance_report', '/vlm_surveillance/phi35_vision_compliance_report'),
            ('scene_analysis', '/vlm_surveillance/phi35_vision_scene_analysis')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # System initialization info
    system_info = LogInfo(
        msg='\n========================================\n'
            '🧠 PHI-3.5 VISION COMPREHENSIVE SURVEILLANCE SYSTEM LAUNCHED\n'
            '========================================\n'
            '🔍 Model: Phi-3.5-vision-instruct (4.2B parameters)\n'
            '🎯 Capabilities: State-of-the-art multi-modal reasoning\n'
            '❓ Advanced VQA: Enabled\n'
            '⚖️  Compliance Monitoring: Enabled\n'
            '🚨 Threat Assessment: Comprehensive\n'
            '📊 Analysis Type: Full spectrum surveillance\n'
            '🔧 Optimization: Flash attention + Mixed precision\n'
            '========================================\n'
            '📡 Subscribed Topics:\n'
            '   • /camera/image_raw (sensor_msgs/Image)\n'
            '📢 Published Topics:\n'
            '   • /vlm_surveillance/phi35_vision_alert (std_msgs/String)\n'
            '   • /vlm_surveillance/omnivlm_threat_assessment (std_msgs/String)\n'
            '   • /vlm_surveillance/omnivlm_compliance_report (std_msgs/String)\n'
            '   • /vlm_surveillance/omnivlm_scene_analysis (std_msgs/String)\n'
            '🛠️  Available Services:\n'
            '   • /vlm_surveillance/omnivlm_analysis\n'
            '   • /vlm_surveillance/scene_understanding\n'
            '   • /vlm_surveillance/advanced_vqa\n'
            '   • /vlm_surveillance/compliance_assessment\n'
            '========================================\n'
            '🚀 OmniVLM-968M Surveillance System Ready!\n'
            '   Processing images with comprehensive analysis...\n'
            '========================================'
    )
    
    # Launch description with all components
    return LaunchDescription([
        # Launch arguments
        threat_sensitivity_arg,
        surveillance_frequency_arg,
        log_level_arg,
        use_advanced_analysis_arg,
        compliance_monitoring_arg,
        
        # System information
        system_info,
        
        # Core nodes
        phi35_vision_service_node,
        phi35_vision_surveillance_node
    ])


if __name__ == '__main__':
    generate_launch_description()
