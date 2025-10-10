#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    Launch the Heterogeneous USV-UAV Mission Planning System
    """
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get source directory for python files
    src_dir = '/home/muhayy/mbzirc_ws/src/unified_mission_planner/unified_mission_planner'
    
    # Heterogeneous Mission Planner Process
    heterogeneous_planner_process = ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'heterogeneous_mission_planner.py')],
        name='heterogeneous_mission_planner',
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        
        LogInfo(msg="🤖 Starting Heterogeneous USV-UAV Mission Planning System"),
        LogInfo(msg="🚢 USV: Maritime navigation and surface inspection capabilities"),
        LogInfo(msg="🚁 UAV: Aerial surveillance and overhead inspection capabilities"),
        LogInfo(msg="🔗 Coordinated: Dependency-based execution with precondition management"),
        LogInfo(msg="🧠 AI-Powered: GPT-4 generates coordinated mission plans"),
        LogInfo(msg="📊 Mathematical: Symbolic action framework τᵢ = aᵢ(robot, {θᵢ}, {σᵢ})"),
        LogInfo(msg="🎯 Compatible: Works with existing USV navigator and UAV executor"),
        
        heterogeneous_planner_process,
    ])
