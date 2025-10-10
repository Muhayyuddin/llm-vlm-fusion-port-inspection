# Unified Mission Planner for USV-UAV Heterogeneous System

## Overview

The Unified Mission Planner provides coordinated mission planning and execution for heterogeneous USV (Unmanned Surface Vehicle) and UAV (Unmanned Aerial Vehicle) systems. It generates structured mission plans using a single LLM call and coordinates execution across both platforms.

## Architecture

### Components

1. **Unified Mission Planner** (`unified_mission_planner.py`)
   - Generates coordinated mission plans using LLM
   - Creates structured JSON plans with symbolic actions
   - Single LLM call for both USV and UAV coordination

2. **Mission Coordinator** (`mission_coordinator.py`)
   - Manages mission execution and dependencies
   - Handles synchronization between USV and UAV
   - Tracks mission progress and status

3. **Existing Executors** (kept unchanged)
   - USV Navigator: `nav_packages/navigation/navigation/navigator.py`
   - UAV Executor: `uav_llm_mission_planner/uav_llm_mission_planner/uav_mission_executor.py`

## Mission Format

### Symbolic Action Set
```
𝒜 = {Takeoff, FlyTo, Survey, Record, Hover, Navigate, Dock, LandOnUSV, Inspect, Report, GoHome}
```

### Action Definition
Each symbolic action is defined as:
```
τᵢ = aᵢ(robot, {θᵢ}, {σᵢ})
```

Where:
- `aᵢ ∈ 𝒜`: Action type
- `θᵢ`: Geometric and temporal parameters (positions, patterns, durations) - **shown in blue**
- `σᵢ`: Sensor configurations and inspection criteria - **shown in red**

### Example Mission Plan

```json
[
  {
    "action": "Navigate",
    "robot": "USV",
    "theta": {"position": {"x": 1400, "y": 20}, "speed": 3.0, "orientation": 1.57},
    "sigma": {}
  },
  {
    "action": "Takeoff",
    "robot": "UAV",
    "theta": {"location": "USVDeck", "altitude": 15, "duration": 30},
    "sigma": {}
  },
  {
    "action": "FlyTo",
    "robot": "UAV",
    "theta": {"position": {"x": 1400, "y": 20}, "altitude": 15, "speed": 10.0},
    "sigma": {}
  },
  {
    "action": "Survey",
    "robot": "UAV",
    "theta": {"pattern": "Orbit360", "altitude": 15, "duration": 300, "radius": 30},
    "sigma": {"sensors": ["camera"], "detection": "Anomaly"}
  },
  {
    "action": "Report",
    "robot": "UAV",
    "theta": {},
    "sigma": {"detection": "Anomaly", "confidence": 0.82}
  },
  {
    "action": "FlyTo",
    "robot": "UAV",
    "theta": {"location": "HoverPointAboveUSV", "altitude": 15},
    "sigma": {}
  },
  {
    "action": "LandOnUSV",
    "robot": "UAV",
    "theta": {"location": "USVDeck"},
    "sigma": {"landing_sensors": ["camera"], "safety_checks": true}
  },
  {
    "action": "GoHome",
    "robot": "USV",
    "theta": {"position": {"x": 0, "y": 0}, "speed": 4.0},
    "sigma": {}
  }
]
```

## Usage

### 1. Launch the Unified System

```bash
# Terminal 1: Launch unified mission planning system
ros2 launch unified_mission_planner unified_mission_system.launch.py

# Terminal 2: Launch USV navigation (existing)
ros2 run navigation navigate

# Terminal 3: Launch UAV executor (existing)
ros2 run uav_llm_mission_planner uav_mission_executor
```

### 2. Send Mission Request

#### Option A: LLM-Generated Mission
```bash
ros2 topic pub --once /mission_request std_msgs/msg/String '{
  data: "{
    \"objective\": \"Survey port crane area for anomalies and return to base\",
    \"area\": {
      \"port_crane_zone\": {\"x\": 1400, \"y\": 20, \"radius\": 50},
      \"home_base\": {\"x\": 0, \"y\": 0}
    }
  }"
}'
```

#### Option B: Predefined Mission Plan
```bash
ros2 run unified_mission_planner mission_test
```

### 3. Monitor Mission Progress

```bash
# Monitor coordination status
ros2 topic echo /mission_coordinator/status

# Monitor USV status
ros2 topic echo /reach_subgoal

# Monitor UAV status
ros2 topic echo /uav/mission_status
```

## Key Features

### 1. **Coordinated Planning**
- Single LLM call generates plans for both USV and UAV
- Maintains spatial and temporal coordination
- Handles dependencies (e.g., UAV takeoff after USV positioning)

### 2. **Structured Execution**
- Dependency graph ensures proper sequencing
- Parallel execution where possible
- Automatic synchronization between platforms

### 3. **Existing Integration**
- Works with current USV navigator
- Compatible with existing UAV executor
- No changes required to execution layers

### 4. **Flexible Mission Definition**
- JSON-based mission format
- Extensible action set
- Clear separation of geometric (θ) and sensor (σ) parameters

## Topics and Communication

### Input Topics
- `/mission_request`: High-level mission objectives (for LLM generation)
- `/unified_mission_plan`: Direct mission plan input

### Output Topics
- `/target_vessel_pose`: USV navigation commands
- `/uav/mission_command`: UAV execution commands

### Status Topics
- `/mission_coordinator/status`: Overall mission progress
- `/reach_subgoal`: USV task completion
- `/uav/mission_status`: UAV task completion

## Configuration

Edit `config/mission_config.yaml` to customize:
- Action definitions
- Robot capabilities
- Coordination parameters
- Communication topics

## Example Missions

See `config/example_missions.json` for:
- Complex survey missions
- Simple coordination tests
- Multi-platform operations

## Dependencies

### Required Packages
- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `openai` (for LLM generation)

### Optional
- Existing USV navigation packages
- Existing UAV mission execution packages

## Testing

```bash
# Test coordination without LLM
ros2 run unified_mission_planner mission_test

# Test with real robots
ros2 launch unified_mission_planner unified_mission_system.launch.py
```

## Benefits

1. **Single Point of Control**: One LLM call coordinates entire mission
2. **Maintains Existing Code**: No changes to USV/UAV executors
3. **Structured Format**: Clear θ/σ parameter separation
4. **Dependency Management**: Automatic sequencing and synchronization
5. **Scalable**: Easy to add new actions and robots
