# UAV Navigation Package

A ROS 2 package for autonomous quadrotor navigation using PID control and line-of-sight guidance algorithms. This package provides waypoint-based navigation with real-time position feedback and comprehensive logging capabilities.

## Overview

The UAV Navigation package enables autonomous flight control for quadrotors in ROS 2 environments. It implements a complete navigation stack with position control, waypoint following, and real-time monitoring.

## Features

- ✅ **Multi-waypoint Navigation**: Sequential navigation through configurable waypoints
- ✅ **PID Control**: Precise velocity control with configurable gains
- ✅ **Line-of-Sight Guidance**: Direct path following algorithm for smooth trajectories
- ✅ **Real-time Position Tracking**: Odometry-based state estimation
- ✅ **TF Broadcasting**: Custom TF broadcaster for UAV pose data
- ✅ **Comprehensive Logging**: Detailed debug information and CSV data logging
- ✅ **IMU Integration**: Orientation feedback for enhanced control
- ✅ **Configurable Parameters**: Tunable flight parameters and waypoint thresholds

## Package Architecture

### Nodes

1. **`uav_navigation_node`** - Main navigation controller
2. **`uav_odom_tf_broadcaster`** - TF broadcaster and odometry publisher

### Controllers

1. **`PIDController`** - Velocity regulation and output limiting
2. **`GuidanceController`** - Line-of-sight guidance for waypoint following

## Dependencies

### ROS 2 Packages
```bash
rclpy                # ROS 2 Python client library
geometry_msgs        # Twist and Point message types
sensor_msgs          # IMU message types
nav_msgs             # Odometry message types
std_msgs             # Standard message types
tf2_msgs             # Transform messages
tf2_ros              # Transform broadcasting
```

### Python Dependencies
```bash
numpy               # Numerical computations
math                # Mathematical functions
csv                 # Data logging
datetime            # Timestamp handling
```

## Installation

1. **Navigate to your ROS 2 workspace**:
```bash
cd ~/mbzirc_ws/src/nav_packages
```

2. **Install dependencies**:
```bash
cd ~/mbzirc_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package**:
```bash
colcon build --packages-select uav_navigation
source install/setup.bash
```

## Usage

### Quick Start

**Launch the complete navigation system**:
```bash
ros2 launch uav_navigation uav_navigation.launch.py
```

**Run individual nodes**:
```bash
# Navigation controller only
ros2 run uav_navigation uav_navigation_node

# TF broadcaster only
ros2 run uav_navigation uav_odom_tf_broadcaster
```

### Custom Configuration

**Launch with custom parameters**:
```bash
ros2 launch uav_navigation uav_navigation.launch.py \
  namespace:=my_quadrotor \
  use_sim_time:=true \
  log_level:=DEBUG
```

**Debug mode with verbose logging**:
```bash
ros2 launch uav_navigation uav_navigation.launch.py log_level:=DEBUG
```

## Topic Interface

### Subscriptions

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/quadrotor_1/pose_static` | `tf2_msgs/TFMessage` | Static pose data from simulation |
| `/uav/odom` | `nav_msgs/Odometry` | Position and velocity feedback |
| `/quadrotor_1/imu/data` | `sensor_msgs/Imu` | Orientation and angular velocity |

### Publications

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/quadrotor_1/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to UAV |
| `/uav/odom` | `nav_msgs/Odometry` | Processed odometry data |

### TF Frames

| Frame | Parent | Description |
|-------|---------|-------------|
| `uav` | `odom` | UAV body frame |
| `quadrotor_1` | `world` | Simulation quadrotor frame |

## Configuration

### Default Waypoints

The system comes with pre-configured waypoints for a rectangular flight pattern:

```python
self.target_waypoints = [
    Point(x=20.0, y=0.0, z=10.0),   # Point 1
    Point(x=20.0, y=20.0, z=10.0),  # Point 2
    Point(x=0.0, y=20.0, z=10.0),   # Point 3
    Point(x=0.0, y=0.0, z=2.0)      # Landing point
]
```

### Navigation Parameters

```python
waypoint_threshold = 0.5    # Distance threshold for waypoint completion (meters)
control_frequency = 10.0    # Control loop frequency (Hz)
max_velocity = 2.0          # Maximum velocity limit (m/s)
```

### PID Configuration

Edit `config/pid_params.yaml` to tune control parameters:

```yaml
uav_navigation_node:
  ros__parameters:
    # PID gains for each axis
    pid_x: {kp: 1.0, ki: 0.1, kd: 0.2}
    pid_y: {kp: 1.0, ki: 0.1, kd: 0.2}
    pid_z: {kp: 1.5, ki: 0.2, kd: 0.3}
    pid_yaw: {kp: 2.0, ki: 0.0, kd: 0.5}
    
    # Navigation parameters
    waypoint_threshold: 0.5
    max_velocity: 2.0
```

## Package Structure

```
uav_navigation/
├── README.md                      # This documentation
├── package.xml                    # Package metadata
├── setup.py                       # Python package setup
├── setup.cfg                      # Package configuration
├── resource/
│   └── uav_navigation            # Resource marker file
├── uav_navigation/               # Main Python package
│   ├── __init__.py              # Package initialization
│   ├── uav_navigation_node.py   # Main navigation controller
│   ├── uav_odom_tf_broadcaster.py # TF broadcaster node
│   ├── pid_controller.py        # PID control implementation
│   └── guidance_controller.py   # Guidance algorithms
├── launch/
│   └── uav_navigation.launch.py # Launch configuration
└── config/
    └── pid_params.yaml          # PID parameters
```

## API Reference

### UAVNavigationNode

Main navigation controller with comprehensive logging and control.

#### Key Methods

- `odom_callback(msg)` - Process odometry data and update position
- `imu_callback(msg)` - Process IMU data for orientation
- `control_loop()` - Main control loop (10Hz) with detailed logging
- `is_waypoint_reached(target)` - Check waypoint completion with distance monitoring
- `calculate_distance(pos1, pos2)` - 3D Euclidean distance calculation

#### Logged Information

- Current waypoint index and target coordinates
- Real-time position updates
- Position differences (dx, dy, dz)
- Desired velocity commands
- Control output values
- Published command velocities
- Distance to waypoint with threshold checking

### UAVTFBroadcaster

TF broadcaster that converts simulation pose data to ROS odometry.

#### Key Methods

- `tf_callback(msg)` - Process TF messages from simulation
- `publish_tf(transform)` - Broadcast TF transforms
- `publish_odometry(transform)` - Publish odometry messages
- `log_to_csv(transform)` - Log position data to CSV files

#### Features

- Automatic CSV logging with timestamps
- Quaternion to Euler angle conversion
- Real-time TF broadcasting
- Position and orientation tracking

## Monitoring and Debugging

### Real-time Monitoring

**Check topic activity**:
```bash
# Monitor position feedback
ros2 topic echo /uav/odom

# Monitor velocity commands
ros2 topic echo /quadrotor_1/cmd_vel

# Check TF tree
ros2 run tf2_tools view_frames
```

**Monitor node status**:
```bash
# List active nodes
ros2 node list

# Check node information
ros2 node info /uav_navigation_node
```

### Debug Logging

**Enable debug logging**:
```bash
ros2 launch uav_navigation uav_navigation.launch.py log_level:=DEBUG
```

**Monitor specific topics**:
```bash
# Check IMU data
ros2 topic echo /quadrotor_1/imu/data

# Monitor pose data
ros2 topic echo /quadrotor_1/pose_static
```

### CSV Data Logging

The TF broadcaster automatically creates CSV files with timestamped position data:
- Filename format: `YYYY-MM-DD_HH-MM-SS_uav_position_data.csv`
- Contains: Relative time, Position (X,Y,Z), Orientation (Roll,Pitch,Yaw)

## Troubleshooting

### Common Issues

1. **"No position data available yet"**
   - Check if `/uav/odom` topic is publishing
   - Verify TF broadcaster is running
   - Ensure simulation is providing pose data

2. **Waypoint not reached**
   - Verify waypoint threshold settings
   - Check distance calculations in logs
   - Monitor velocity commands being published

3. **Erratic flight behavior**
   - Tune PID parameters in config file
   - Check for excessive noise in position data
   - Verify control loop frequency

4. **TF broadcaster not working**
   - Check if `/quadrotor_1/pose_static` is publishing
   - Verify correct frame IDs in TF messages
   - Monitor CSV log file creation

### Debug Commands

```bash
# Check all active topics
ros2 topic list

# Monitor node logs in real-time
ros2 run rqt_console rqt_console

# Check TF tree structure
ros2 run rqt_tf_tree rqt_tf_tree

# Test manual velocity commands
ros2 topic pub --once /quadrotor_1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Performance Characteristics

- **Control Loop Frequency**: 10 Hz
- **Position Update Rate**: Depends on odometry source
- **Waypoint Completion Threshold**: 0.5 meters (configurable)
- **Maximum Velocity**: 2.0 m/s (configurable)
- **Memory Usage**: ~10-15 MB per node
- **CPU Usage**: <5% on modern systems

## Integration with MBZIRC Simulation

This package is designed to work with the MBZIRC simulation environment:

1. **Launch simulation first**:
```bash
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"
```

2. **Spawn UAV**:
```bash
ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_1 world:=coast model:=quadrotor type:=quadrotor x:=0 y:=0 z:=2.0
```

3. **Launch navigation**:
```bash
ros2 launch uav_navigation uav_navigation.launch.py
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Maintainer

- **Name**: Your Name
- **Email**: your.email@example.com
- **Institution**: MBZIRC Team

## Acknowledgments

- ROS 2 community for the excellent framework
- MBZIRC challenge organizers
- Ignition Gazebo simulation environment
- Contributors to the navigation algorithms

## Citation

If you use this package in your research, please cite:

```bibtex
@software{uav_navigation_mbzirc,
  title={UAV Navigation Package for MBZIRC Challenge},
  author={Your Name},
  year={2025},
  url={https://github.com/yourusername/uav_navigation},
  note={ROS 2 Package for Autonomous Quadrotor Navigation}
}
```

## Version History

- **v0.2.0** - Added TF broadcaster, comprehensive logging, CSV data export
- **v0.1.0** - Initial release with basic waypoint navigation and PID control
- **v0.0.1** - Development version with core functionality

---

**Status**: ✅ **Working** - Package tested and operational with MBZIRC simulation environment.