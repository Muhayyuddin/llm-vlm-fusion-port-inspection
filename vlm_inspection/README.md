# VLM Inspection Package

A ROS2 package for visual language model inspection using Microsoft's Florence-2 and Salesforce's BLIP2 models.

## Overview

This package provides Florence-2 and BLIP2 integration for ROS2, allowing you to generate detailed descriptions of images using state-of-the-art vision-language models. The package includes both ROS2 nodes for real-time image processing and standalone scripts for batch processing.

## Features

- **Florence-2 Integration**: Uses Microsoft's Florence-2 model for comprehensive image analysis
- **BLIP2 Integration**: Uses Salesforce's BLIP2 model for image description and question answering
- **ROS2 Nodes**: Real-time image processing via ROS2 topics
- **Standalone Scripts**: Command-line tools for single image processing
- **Multiple Tasks**: Supports various tasks (captioning, object detection, OCR, Q&A)
- **Configurable**: Easy parameter configuration for different models and tasks
- **Dual Model Support**: Run both models simultaneously or independently

## Installation

### Prerequisites

Make sure you have ROS2 installed and a workspace set up.

### Dependencies

Install the required Python packages:

```bash
pip install torch transformers pillow timm einops
```

### Build the Package

```bash
cd ~/vlm_ws
colcon build --packages-select vlm_inspection
source install/setup.bash
```

## Usage

### ROS2 Nodes

#### Launch Florence-2 Node

```bash
ros2 launch vlm_inspection florence_launch.py
```

#### Launch BLIP2 Node

```bash
ros2 launch vlm_inspection blip2_launch.py
```

#### Launch Both Models

```bash
# Launch both models
ros2 launch vlm_inspection vlm_dual_launch.py enable_florence:=true enable_blip2:=true

# Launch only Florence-2
ros2 launch vlm_inspection vlm_dual_launch.py enable_florence:=true enable_blip2:=false

# Launch only BLIP2
ros2 launch vlm_inspection vlm_dual_launch.py enable_florence:=false enable_blip2:=true
```

#### Florence-2 Parameters

- `model_name`: Florence-2 model to use (default: "microsoft/Florence-2-large")
- `default_task`: Default task for image description (default: "<MORE_DETAILED_CAPTION>")
- `auto_load_model`: Whether to load model automatically on startup (default: true)

#### BLIP2 Parameters

- `model_name`: BLIP2 model to use (default: "Salesforce/blip2-flan-t5-xl")
- `input_topic`: Input image topic (default: "/camera/image_raw")
- `output_topic`: Output description topic (default: "/blip2/description")
- `question_topic`: Input question topic (default: "/blip2/question")
- `answer_topic`: Output answer topic (default: "/blip2/answer")

#### Topics

**Florence-2 Topics:**
- **Subscribed**: `input_image` (sensor_msgs/Image) → `/camera/image_raw`
- **Published**: `image_description` (std_msgs/String) → `/florence/description`

**BLIP2 Topics:**
- **Subscribed**: 
  - `input_image` (sensor_msgs/Image) → `/camera/image_raw`
  - `question` (std_msgs/String) → `/blip2/question`
- **Published**: 
  - `image_description` (std_msgs/String) → `/blip2/description`
  - `image_answer` (std_msgs/String) → `/blip2/answer`

#### Example Usage

**Florence-2:**
```bash
# Launch Florence-2 node
ros2 launch vlm_inspection florence_launch.py

# In another terminal, publish an image (example with usb_cam)
ros2 run usb_cam usb_cam_node_exe

# Listen to descriptions
ros2 topic echo /florence/description
```

**BLIP2:**
```bash
# Launch BLIP2 node
ros2 launch vlm_inspection blip2_launch.py

# In another terminal, publish an image
ros2 run usb_cam usb_cam_node_exe

# Listen to descriptions
ros2 topic echo /blip2/description

# Ask questions about the image
ros2 topic pub /blip2/question std_msgs/String "data: 'What do you see in this image?'"

# Listen to answers
ros2 topic echo /blip2/answer
```

**Both Models:**
```bash
# Launch both models
ros2 launch vlm_inspection vlm_dual_launch.py enable_florence:=true enable_blip2:=true

# Compare descriptions
ros2 topic echo /florence/description &
ros2 topic echo /blip2/description &
```

### Standalone Script

The package also provides a standalone script for processing individual images:

```bash
# Basic usage
ros2 run vlm_inspection image_describer /path/to/your/image.jpg

# With custom task
ros2 run vlm_inspection image_describer /path/to/your/image.jpg --task "<CAPTION>"

# With different model
ros2 run vlm_inspection image_describer /path/to/your/image.jpg --model "microsoft/Florence-2-base"

# Verbose output
ros2 run vlm_inspection image_describer /path/to/your/image.jpg --verbose
```

## Available Models

### Florence-2 Tasks

- `<CAPTION>`: Basic image caption
- `<DETAILED_CAPTION>`: Detailed image caption
- `<MORE_DETAILED_CAPTION>`: Very detailed image caption (default)
- `<OD>`: Object detection
- `<DENSE_REGION_CAPTION>`: Dense region captioning
- `<REGION_PROPOSAL>`: Region proposal
- `<OCR>`: Optical character recognition
- `<OCR_WITH_REGION>`: OCR with region information

### BLIP2 Models

- `Salesforce/blip2-opt-2.7b`: BLIP2 with OPT-2.7B (faster, smaller)
- `Salesforce/blip2-opt-6.7b`: BLIP2 with OPT-6.7B
- `Salesforce/blip2-flan-t5-xl`: BLIP2 with Flan-T5-XL (default, good balance)
- `Salesforce/blip2-flan-t5-xxl`: BLIP2 with Flan-T5-XXL (largest, best quality)

## Configuration

### Launch File Parameters

You can customize the behavior by passing parameters to the launch file:

```bash
ros2 launch vlm_inspection florence_launch.py model_name:=microsoft/Florence-2-base default_task:="<CAPTION>"
```

### Node Parameters

```bash
ros2 run vlm_inspection florence_node --ros-args -p model_name:=microsoft/Florence-2-base -p default_task:="<CAPTION>"
```

## Performance Notes

- **GPU Acceleration**: The package automatically uses CUDA if available
- **Model Loading**: The large model (~2GB) takes time to load initially
- **Memory Usage**: Ensure sufficient GPU/RAM memory for the model
- **First Inference**: The first inference may take longer due to model initialization

## Troubleshooting

### Model Loading Issues

If you encounter model loading issues:

1. Check internet connection (models are downloaded on first use)
2. Ensure sufficient disk space (~2GB for Florence-2-large)
3. Verify PyTorch installation with CUDA support if using GPU

### Memory Issues

If you run out of memory:

1. Use the smaller "microsoft/Florence-2-base" model
2. Reduce image resolution before processing
3. Close other applications to free up memory

### ROS2 Topic Issues

Check topic connections:

```bash
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic info /florence/description
```

## Example Integration

Here's a simple example of how to use the package in your own ROS2 node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ImageAnalyzer(Node):
    def __init__(self):
        super().__init__('image_analyzer')
        
        # Subscribe to Florence descriptions
        self.subscription = self.create_subscription(
            String,
            '/florence/description',
            self.description_callback,
            10
        )
    
    def description_callback(self, msg):
        self.get_logger().info(f'Received description: {msg.data}')
        # Process the description...
```

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.
