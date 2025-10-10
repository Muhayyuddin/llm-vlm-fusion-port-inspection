#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for Florence-2 VLM inspection node
    """
    
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='microsoft/Florence-2-large',
        description='Florence-2 model name to use'
    )
    
    default_task_arg = DeclareLaunchArgument(
        'default_task',
        default_value='<MORE_DETAILED_CAPTION>',
        description='Default task for image description'
    )
    
    auto_load_model_arg = DeclareLaunchArgument(
        'auto_load_model',
        default_value='true',
        description='Whether to automatically load the model on startup'
    )
    
    # Florence node
    florence_node = Node(
        package='vlm_inspection',
        executable='florence_node',
        name='florence_node',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'default_task': LaunchConfiguration('default_task'),
            'auto_load_model': LaunchConfiguration('auto_load_model')
        }],
        remappings=[
            ('input_image', 'camera/image_raw'),
            ('image_description', 'florence/description')
        ]
    )
    
    return LaunchDescription([
        model_name_arg,
        default_task_arg,
        auto_load_model_arg,
        florence_node
    ])
