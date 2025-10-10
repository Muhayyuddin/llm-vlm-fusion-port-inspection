#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Launch file for both Florence-2 and BLIP2 image description nodes
    """
    
    # Declare launch arguments
    enable_florence_arg = DeclareLaunchArgument(
        'enable_florence',
        default_value='true',
        description='Enable Florence-2 node'
    )
    
    enable_blip2_arg = DeclareLaunchArgument(
        'enable_blip2',
        default_value='false',
        description='Enable BLIP2 node'
    )
    
    florence_model_arg = DeclareLaunchArgument(
        'florence_model',
        default_value='microsoft/Florence-2-large',
        description='Florence-2 model to use'
    )
    
    blip2_model_arg = DeclareLaunchArgument(
        'blip2_model',
        default_value='Salesforce/blip2-opt-2.7b',
        description='BLIP2 model to use'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',  
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    # Florence-2 Node
    florence_node = Node(
        package='vlm_inspection',
        executable='florence_node',
        name='florence_node',
        parameters=[{
            'model_name': LaunchConfiguration('florence_model'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/florence/description'
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_florence'))
    )
    
    # BLIP2 Node
    blip2_node = Node(
        package='vlm_inspection',
        executable='blip2_node',
        name='blip2_node',
        parameters=[{
            'model_name': LaunchConfiguration('blip2_model'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/blip2/description',
            'question_topic': '/blip2/question',
            'answer_topic': '/blip2/answer'
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_blip2'))
    )
    
    return LaunchDescription([
        enable_florence_arg,
        enable_blip2_arg,
        florence_model_arg,
        blip2_model_arg,
        input_topic_arg,
        florence_node,
        blip2_node
    ])
