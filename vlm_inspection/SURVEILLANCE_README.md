# Surveillance Node Documentation

## Overview
The surveillance node subscribes to a quadrotor camera feed and performs real-time image analysis using Florence-2 vision-language model to detect suspicious activities.

## Features
- **Real-time Analysis**: Processes camera images at 1 Hz (configurable)
- **Florence-2 Integration**: Uses Microsoft's Florence-2-large model for detailed image understanding
- **Suspicious Activity Detection**: Keyword-based analysis for potential security concerns
- **Formatted Reporting**: Structured surveillance reports with timestamps
- **Configurable Parameters**: Customizable topics, models, and analysis rates

## Usage

### Basic Launch
```bash
# Source your ROS2 workspace
source install/setup.bash

# Launch surveillance node with default settings
ros2 launch vlm_inspection surveillance_launch.py
```

### Custom Configuration
```bash
# Launch with custom quadrotor topic
ros2 launch vlm_inspection surveillance_launch.py input_topic:=/custom_drone/camera/image

# Launch with different analysis rate (0.5 Hz = every 2 seconds)
ros2 launch vlm_inspection surveillance_launch.py analysis_rate:=0.5

# Launch with smaller Florence model for faster processing
ros2 launch vlm_inspection surveillance_launch.py model_name:=microsoft/Florence-2-base
```

### Manual Node Execution
```bash
# Run surveillance node directly
ros2 run vlm_inspection surveillance_node
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/quadrotor_1/slot0/image_raw` | Camera image topic to monitor |
| `model_name` | string | `microsoft/Florence-2-large` | Florence-2 model variant |
| `analysis_rate` | float | `1.0` | Analysis frequency in Hz |

## Surveillance Report Format

The node generates structured reports with the following format:

```
================================================================================
🚁 SURVEILLANCE REPORT - [TIMESTAMP]
================================================================================
📍 PROMPT: I am surveying the area. This image is captured by my camera.
           Is there anything suspicious in the image?
--------------------------------------------------------------------------------
🔍 ANALYSIS RESULT:
   [Detailed Florence-2 image description]
--------------------------------------------------------------------------------
[THREAT ASSESSMENT]
⚠️  POTENTIAL CONCERNS DETECTED: [keywords]
✅ NO IMMEDIATE CONCERNS DETECTED: Area appears normal
================================================================================
```

## Suspicious Activity Keywords

The node monitors for the following keywords in image descriptions:
- **Weapons**: weapon, gun, knife
- **People**: suspicious, person, people, running, hiding
- **Vehicles**: vehicle, car, truck, moving
- **Environmental**: fire, smoke, damage, broken, unusual, strange

## Technical Requirements

### Dependencies
- ROS2 (tested with ROS2 Humble)
- Python 3.8+
- PyTorch 2.4.1+ with CUDA support
- Transformers library
- PIL (Pillow)
- sensor_msgs for image handling

### Hardware Requirements
- **GPU Memory**: Minimum 4GB VRAM for Florence-2-large
- **System RAM**: Minimum 8GB
- **Storage**: ~2GB for model weights

### Performance
- **Analysis Rate**: 1 Hz (configurable, limited by model inference time)
- **Model Load Time**: ~10-30 seconds (first run)
- **Memory Usage**: ~3-4GB GPU memory for Florence-2-large

## Troubleshooting

### Common Issues

1. **GPU Memory Error**
   ```
   Solution: Use smaller model variant:
   ros2 launch vlm_inspection surveillance_launch.py model_name:=microsoft/Florence-2-base
   ```

2. **No Image Messages**
   ```
   Check topic is publishing:
   ros2 topic echo /quadrotor_1/slot0/image_raw --once
   
   Verify topic name:
   ros2 topic list | grep image
   ```

3. **Model Loading Slow**
   ```
   First run downloads model weights (~1.5GB)
   Subsequent runs use cached weights
   ```

### Debug Mode
```bash
# Launch with debug output
ros2 launch vlm_inspection surveillance_launch.py --debug
```

## Integration Example

### With Quadrotor Simulation
```bash
# Terminal 1: Start quadrotor simulation
ros2 launch your_simulation quadrotor_sim.launch.py

# Terminal 2: Start surveillance monitoring
ros2 launch vlm_inspection surveillance_launch.py
```

### Multi-Drone Surveillance
```bash
# Monitor multiple quadrotors
ros2 launch vlm_inspection surveillance_launch.py input_topic:=/quadrotor_1/slot0/image_raw &
ros2 launch vlm_inspection surveillance_launch.py input_topic:=/quadrotor_2/slot0/image_raw &
```

## Development

### Code Structure
- `surveillance_node.py`: Main ROS2 node implementation
- `florence_service.py`: Florence-2 model integration
- `surveillance_launch.py`: Launch configuration

### Testing
```bash
# Run surveillance functionality test
cd src/vlm_inspection
python3 test_surveillance.py
```

### Extending Functionality
The surveillance node can be extended with:
- Custom threat detection algorithms
- Integration with other VLM models (BLIP2, LLaVA)
- Real-time alerts and notifications
- Database logging of surveillance events
- Multi-camera fusion and tracking

## Safety Considerations
- This tool is for educational and research purposes
- Ensure compliance with local privacy and surveillance regulations
- Consider ethical implications of automated surveillance systems
- Test thoroughly in controlled environments before deployment
