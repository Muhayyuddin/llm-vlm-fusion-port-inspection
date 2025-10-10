#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch the unified mission planning system
    """
    
    # Package directory
    package_dir = get_package_share_directory('unified_mission_planner')
    config_dir = os.path.join(package_dir, 'config')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_dir, 'mission_config.yaml'),
        description='Path to mission configuration file'
    )
    
    # Get source directory for python files
    src_dir = '/home/muhayy/mbzirc_ws/src/unified_mission_planner/unified_mission_planner'
    
    # Unified Mission Planner Process
    unified_planner_process = ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'unified_mission_planner.py')],
        name='unified_mission_planner',
        output='screen'
    )
    
    # Mission Coordinator Process
    mission_coordinator_process = ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'mission_coordinator.py')],
        name='mission_coordinator',
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        
        LogInfo(msg="🚀 Starting Unified Mission Planning System"),
        LogInfo(msg="📋 Mission Planner: Generates coordinated USV-UAV plans"),
        LogInfo(msg="🎯 Mission Coordinator: Manages execution and dependencies"),
        LogInfo(msg="🔗 Integration: Works with existing USV and UAV executors"),
        
        unified_planner_process,
        mission_coordinator_process,
    ])
