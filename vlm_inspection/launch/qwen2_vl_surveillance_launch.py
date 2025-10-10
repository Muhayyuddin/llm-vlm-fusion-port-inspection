#!/usr/bin/env python3
"""
Launch file for Qwen2-VL Comprehensive Surveillance System

This launch file deploys the Qwen2-VL surveillance service and node
for advanced vision-language understanding, threat assessment, and 
multilingual surveillance monitoring.

Qwen2-VL Features:
- Advanced vision-language understanding with excellent reasoning
- Multilingual support (English, Chinese, bilingual)
- Efficient 2B/7B parameter models 
- Comprehensive surveillance analysis
- Real-time threat detection and assessment
- Regulatory compliance monitoring
- Multi-perspective scene understanding

Author: VLM Inspection Team
Date: 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for Qwen2-VL comprehensive surveillance system
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # Declare launch arguments for advanced configuration
    model_variant_arg = DeclareLaunchArgument(
        'model_variant',
        default_value='Qwen/Qwen2-VL-2B-Instruct',
        description='Qwen2-VL model variant: Qwen/Qwen2-VL-2B-Instruct, Qwen/Qwen2-VL-7B-Instruct, Qwen/Qwen2.5-VL-2B-Instruct'
    )
    
    language_mode_arg = DeclareLaunchArgument(
        'language_mode',
        default_value='english',
        description='Language mode: english, chinese, bilingual'
    )
    
    threat_sensitivity_arg = DeclareLaunchArgument(
        'threat_sensitivity',
        default_value='medium',
        description='Threat detection sensitivity level: low, medium, high'
    )
    
    surveillance_frequency_arg = DeclareLaunchArgument(
        'surveillance_frequency',
        default_value='1.0',
        description='Image processing frequency in Hz (recommended: 0.5-2.0 for Qwen2-VL)'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/quadrotor_1/slot0/image_raw',
        description='Input camera topic for surveillance'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='ROS2 logging level: DEBUG, INFO, WARN, ERROR'
    )
    
    use_advanced_analysis_arg = DeclareLaunchArgument(
        'use_advanced_analysis',
        default_value='true',
        description='Enable advanced multi-perspective scene understanding'
    )
    
    compliance_monitoring_arg = DeclareLaunchArgument(
        'compliance_monitoring',
        default_value='true',
        description='Enable regulatory compliance monitoring'
    )
    
    advanced_vqa_arg = DeclareLaunchArgument(
        'advanced_vqa',
        default_value='true',
        description='Enable advanced visual question answering'
    )
    
    # Qwen2-VL Service Node - Advanced AI Backend
    qwen2_vl_service_node = Node(
        package='vlm_inspection',
        executable='qwen2_vl_service',
        name='qwen2_vl_service_node',
        namespace='vlm_surveillance',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_variant'),
            'auto_load_model': True,
            'max_new_tokens': 512,
            'temperature': 0.0,
            'do_sample': False,
            'memory_optimization': True,
            'torch_dtype': 'float16',
            'low_cpu_mem_usage': True
        }],
        remappings=[
            ('qwen2_vl_analysis', '/vlm_surveillance/qwen2_vl_analysis'),
            ('scene_understanding', '/vlm_surveillance/qwen2_vl_scene_understanding'),
            ('surveillance_anomalies', '/vlm_surveillance/qwen2_vl_surveillance_anomalies'),
            ('compliance_assessment', '/vlm_surveillance/qwen2_vl_compliance_assessment')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Qwen2-VL Comprehensive Surveillance Node - Advanced Monitoring
    qwen2_vl_surveillance_node = Node(
        package='vlm_inspection',
        executable='qwen2_vl_surveillance_node',
        name='qwen2_vl_comprehensive_surveillance',
        namespace='vlm_surveillance',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'model_name': LaunchConfiguration('model_variant'),
            'analysis_rate': LaunchConfiguration('surveillance_frequency'),
            'language_mode': LaunchConfiguration('language_mode'),
            'threat_sensitivity': LaunchConfiguration('threat_sensitivity'),
            'enable_compliance_check': LaunchConfiguration('compliance_monitoring'),
            'enable_scene_understanding': LaunchConfiguration('use_advanced_analysis'),
            'enable_advanced_vqa': LaunchConfiguration('advanced_vqa'),
            'confidence_threshold': 0.8,
            'max_analysis_tokens': 512,
            'enable_detailed_reporting': True,
            'multilingual_support': True,
            'advanced_reasoning': True
        }],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('input_topic')),
            ('surveillance_alert', '/vlm_surveillance/qwen2_vl_alert'),
            ('threat_assessment', '/vlm_surveillance/qwen2_vl_threat_assessment'),
            ('compliance_report', '/vlm_surveillance/qwen2_vl_compliance_report'),
            ('scene_analysis', '/vlm_surveillance/qwen2_vl_scene_analysis'),
            ('vqa_results', '/vlm_surveillance/qwen2_vl_vqa_results')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # System initialization info
    system_info = LogInfo(
        msg='\n========================================\n'
            '🧠 QWEN2-VL COMPREHENSIVE SURVEILLANCE SYSTEM LAUNCHED\n'
            '========================================\n'
            '🔍 Model: Qwen2-VL (Advanced Vision-Language Model)\n'
            '🎯 Capabilities: Excellent vision-language understanding\n'
            '🌐 Language Support: English, Chinese, Bilingual\n'
            '❓ Advanced VQA: Enabled\n'
            '⚖️  Compliance Monitoring: Enabled\n'
            '🚨 Threat Assessment: Comprehensive\n'
            '📊 Analysis Type: Multi-perspective surveillance\n'
            '🔧 Optimization: Memory efficient + Fast inference\n'
            '========================================\n'
            '🛠️  Configuration:\n'
            '   • Model Variant: Dynamic (configurable)\n'
            '   • Language Mode: Dynamic (configurable)\n'
            '   • Threat Sensitivity: Dynamic (configurable)\n'
            '   • Analysis Rate: Dynamic (configurable)\n'
            '========================================\n'
            '📡 Subscribed Topics:\n'
            '   • /quadrotor_1/slot0/image_raw (sensor_msgs/Image)\n'
            '📢 Published Topics:\n'
            '   • /vlm_surveillance/qwen2_vl_alert (std_msgs/String)\n'
            '   • /vlm_surveillance/qwen2_vl_threat_assessment (std_msgs/String)\n'
            '   • /vlm_surveillance/qwen2_vl_compliance_report (std_msgs/String)\n'
            '   • /vlm_surveillance/qwen2_vl_scene_analysis (std_msgs/String)\n'
            '   • /vlm_surveillance/qwen2_vl_vqa_results (std_msgs/String)\n'
            '🛠️  Available Services:\n'
            '   • /vlm_surveillance/qwen2_vl_analysis\n'
            '   • /vlm_surveillance/qwen2_vl_scene_understanding\n'
            '   • /vlm_surveillance/qwen2_vl_surveillance_anomalies\n'
            '   • /vlm_surveillance/qwen2_vl_compliance_assessment\n'
            '========================================\n'
            '🚀 Qwen2-VL Surveillance System Ready!\n'
            '   Processing images with advanced reasoning...\n'
            '========================================'
    )
    
    # Launch description with all components
    return LaunchDescription([
        # Launch arguments
        model_variant_arg,
        language_mode_arg,
        threat_sensitivity_arg,
        surveillance_frequency_arg,
        input_topic_arg,
        log_level_arg,
        use_advanced_analysis_arg,
        compliance_monitoring_arg,
        advanced_vqa_arg,
        
        # System information
        system_info,
        
        # Core nodes
        qwen2_vl_service_node,
        qwen2_vl_surveillance_node
    ])


if __name__ == '__main__':
    generate_launch_description()