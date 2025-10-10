#!/bin/bash

# VLM Inspection Package Setup Script
# This script helps set up the environment for the Florence-2 ROS2 package

set -e  # Exit on error

echo "VLM Inspection Package Setup"
echo "============================"

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "Error: This script should be run from the vlm_inspection package directory"
    echo "cd /home/muhayy/vlm_ws/src/vlm_inspection"
    exit 1
fi

echo "Installing Python dependencies..."

# Check if pip is available
if ! command -v pip &> /dev/null; then
    echo "Error: pip is not installed. Please install pip first."
    exit 1
fi

# Install requirements
if [ -f "requirements.txt" ]; then
    echo "Installing from requirements.txt..."
    pip install -r requirements.txt
else
    echo "Installing individual packages..."
    pip install torch transformers pillow accelerate
fi

echo "Dependencies installed successfully!"

# Test the installation
echo ""
echo "Testing installation..."
python3 test_installation.py

echo ""
echo "Setup complete!"
echo ""
echo "Next steps:"
echo "1. Build the package:"
echo "   cd /home/muhayy/vlm_ws"
echo "   colcon build --packages-select vlm_inspection"
echo ""
echo "2. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "3. Launch the Florence node:"
echo "   ros2 launch vlm_inspection florence_launch.py"
echo ""
echo "4. Test with an image (in another terminal):"
echo "   ros2 run vlm_inspection image_describer /path/to/your/image.jpg"
echo ""
echo "For more information, see README.md"
