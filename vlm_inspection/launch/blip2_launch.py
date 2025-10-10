#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for BLIP2 image description node
    """
    
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='Salesforce/blip2-opt-2.7b',
        description='BLIP2 model to use for image description'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',  
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/blip2/description', 
        description='Output description topic'
    )
    
    question_topic_arg = DeclareLaunchArgument(
        'question_topic',
        default_value='/blip2/question',
        description='Input question topic'
    )
    
    answer_topic_arg = DeclareLaunchArgument(
        'answer_topic',
        default_value='/blip2/answer',
        description='Output answer topic'
    )
    
    # BLIP2 Node
    blip2_node = Node(
        package='vlm_inspection',
        executable='blip2_node',
        name='blip2_node',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'question_topic': LaunchConfiguration('question_topic'),
            'answer_topic': LaunchConfiguration('answer_topic')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_name_arg,
        input_topic_arg,
        output_topic_arg,
        question_topic_arg,
        answer_topic_arg,
        blip2_node
    ])
