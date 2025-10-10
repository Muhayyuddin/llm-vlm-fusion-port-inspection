# LLM-VLM Fusion: Intelligent Maritime Autonomous Systems

## 🌊 Overview

The **LLM-VLM Fusion** project is an advanced autonomous maritime system that integrates Large Language Models (LLM) and Vision-Language Models (VLM) for intelligent control and coordination of heterogeneous USV-UAV operations. This system combines natural language understanding, computer vision, and autonomous navigation to enable sophisticated maritime missions including surveillance, inspection, and survey operations.

## �️ Core Capabilities

<div align="center">
<table>
<tr>
<td align="center" width="50%">

### 🧭 Mapping & SLAM
<img src="assets/mapping.gif" alt="Real-time SLAM and Mapping" width="400"/>

**Real-time Maritime SLAM**
- Simultaneous Localization and Mapping
- Obstacle detection and avoidance


</td>
<td align="center" width="50%">

### 🚢 USV Navigation
<img src="assets/usv-navigation.gif" alt="USV Autonomous Navigation" width="400"/>

**Autonomous Navigation System**
- Waypoint following and trajectory tracking
- Real-time obstacle avoidance

</td>
</tr>
</table>
</div>

## �🚀 System Architecture

This project implements a comprehensive autonomous maritime platform featuring:

- **🤖 LLM-based Mission Planning**: Natural language mission interpretation and execution
- **👁️ VLM Integration**: Vision-language understanding for environmental awareness
- **� Real-time Visual Inspection**: Florence, Quen2VL, SmolVLM-powered analysis
- **�🚢 USV Autonomous Navigation**: Advanced surface vessel control and guidance
- **🚁 UAV Coordination**: Aerial vehicle integration for comprehensive coverage
- **🔗 Heterogeneous Coordination**: Seamless USV-UAV collaborative operations

## 📁 Project Structure

### 📂 `nav2/` - USV SLAM & Navigation
Advanced SLAM and autonomous navigation system for maritime operations with real-time mapping, obstacle avoidance, and ROS2 Nav2 integration.

### 📂 `nav_packages/` - USV Control & Guidance
Low-level control systems featuring PID motion control, dynamic positioning, thrust allocation, and maritime dynamics modeling.

### 📂 `uav_navigation/` - UAV Control & Guidance
Comprehensive UAV flight control with mission execution, inspection patterns, survey operations, and USV deck landing capabilities.

### 📂 `vlm_inspection/` - Vision-Language Model Inspection
Multi-model VLM system (Florence, Quen2VL, SmolVLM) for real-time camera feed analysis, object detection, and automated inspection reports.

### 📂 `unified_mission_planner/` - LLM-based Mission Planning
Intelligent mission planning system using GPT-4 for natural language mission interpretation and heterogeneous USV-UAV coordination.
Features symbolic mission planning, dependency management, survey operations, and coordinated execution with preconditions.

## 🛠️ Installation & Setup

### Prerequisites
- **ROS2 Galactic** or newer
- **Python 3.8+**
- **OpenAI API key** (for LLM integration)
- **Ignition Gazebo** (for simulation)

### Build Instructions
```bash
# Navigate to workspace
cd /path/to/your/workspace

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
colcon build --packages-select nav2 nav_packages uav_navigation unified_mission_planner vlm_inspection

# Source the workspace
source install/setup.bash
```

## 🚀 Quick Start

### 1. Launch Maritime Simulation Environment
```bash
# Start the maritime simulation with coast-port environment
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast-port.sdf"
```

### 2. Spawn USV with Sensors
```bash
# Spawn USV with LiDAR and RGBD camera
ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_planar_lidar slot1:=mbzirc_rgbd_camera

# Alternative: Spawn USV with HD camera and LiDAR
ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera slot1:=mbzirc_planar_lidar
```

### 3. Launch USV Systems
```bash
# Launch USV description and transforms
ros2 launch usv_description usv.launch.py

# Start mapping system
ros2 launch ros2_mapping map_bringsup.launch.py

# Start USV odometry publisher
python3 ~/mbzirc_ws/src/nav_packages/navigation/navigation/usv_odometry_publisher.py
```

### 4. Launch UAV Systems
```bash
# Start UAV transform publisher
ros2 run uav_llm_mission_planner uav_tf_publisher 

# Start UAV mission executor
ros2 run uav_llm_mission_planner uav_mission_executor 
```

### 5. Launch USV Control Systems
```bash
# Set Python path for USV control
export PYTHONPATH=$PYTHONPATH:~/mbzirc_ws/src/nav_packages/usv_control/src

# Start USV twist publisher
ros2 run usv_control twist_publisher 

# Start USV navigator
python3 /home/muhayy/mbzirc_ws/src/nav_packages/navigation/navigation/navigator.py
```

### 6. Start Intelligent Mission Planner
```bash
# Launch LLM-based heterogeneous mission planner
ros2 launch unified_mission_planner heterogeneous_mission_system.launch.py
```

## 🔧 Configuration

### LLM Configuration
Edit `unified_mission_planner/config/openai_key.json`:
```json
{
    "key": "your-openai-api-key-here"
}
```

### Mission Prompts
Customize mission planning in `unified_mission_planner/config/heterogeneous_prompt.txt`

### Navigation Parameters
- USV parameters: `nav_packages/config/usv_navigation_params.yaml`
- UAV parameters: `uav_navigation/config/uav_mission_params.yaml`

## 📊 System Integration

### Communication Architecture
```
┌─────────────────┐    ┌─────────────────┐
│   LLM Mission   │◄──►│   GPT-4 API     │
│    Planner      │    │   Integration   │
└─────────┬───────┘    └─────────────────┘
          │
          ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   USV Control   │◄──►│   UAV Control   │◄──►│  VLM Inspection │
│   & Navigation  │    │   & Navigation  │    │    Module      │
└─────────────────┘    └─────────────────┘    └─────────┬───────┘
          │                       │                      │
          ▼                       ▼                      ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  USV Camera     │    │  UAV Camera     │    │ Florence, Quen2VL│
│  /usv/slot0/    │    │ /quadrotor_1/   │    │   SmolVLM, etc  │
│   image_raw     │    │ slot0/image_raw │    │   VLM Models    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Topic Interface
- **Mission Commands**: `/uav_mission_command`, `/target_vessel_pose`
- **Status Feedback**: `/uav_mission_step_complete`, `/reach_subgoal`
- **Coordination**: `/unified_mission_plan`, `/mission_status`
- **Camera Feeds**: `/usv/slot0/image_raw`, `/quadrotor_1/slot0/image_raw`
- **VLM Analysis**: `/vlm_inspection_results`, `/vlm_detection_annotations`



## 🙏 Acknowledgments

- **ROS2 Community** for the navigation framework
- **OpenAI** for GPT-4 integration capabilities
- **Ignition Gazebo** for simulation environment
- **Maritime Research Community** for domain expertise

