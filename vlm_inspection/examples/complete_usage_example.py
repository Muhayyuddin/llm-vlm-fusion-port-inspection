#!/usr/bin/env python3
"""
Complete example demonstrating the VLM Inspection package usage
This script shows different ways to use the Florence-2 integration
"""

import os
import sys
import time
import subprocess
from pathlib import Path

# Get the package root directory
PACKAGE_ROOT = Path(__file__).parent.parent
ASSETS_DIR = PACKAGE_ROOT / "assets"

def print_header(title):
    """Print a formatted header"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def print_section(title):
    """Print a formatted section header"""
    print(f"\n--- {title} ---")

def check_dependencies():
    """Check if all dependencies are installed"""
    print_header("CHECKING DEPENDENCIES")
    
    try:
        import torch
        print(f"✓ PyTorch {torch.__version__}")
        print(f"  CUDA available: {torch.cuda.is_available()}")
    except ImportError:
        print("✗ PyTorch not installed - run: pip install torch")
        return False
    
    try:
        import transformers
        print(f"✓ Transformers {transformers.__version__}")
    except ImportError:
        print("✗ Transformers not installed - run: pip install transformers")
        return False
    
    try:
        from PIL import Image
        print("✓ Pillow")
    except ImportError:
        print("✗ Pillow not installed - run: pip install pillow")
        return False
    
    try:
        import rclpy
        print("✓ ROS2 rclpy")
    except ImportError:
        print("✗ ROS2 not available")
        return False
    
    return True

def list_sample_images():
    """List available sample images"""
    print_section("Available Sample Images")
    
    if not ASSETS_DIR.exists():
        print("No assets directory found!")
        return []
    
    image_files = list(ASSETS_DIR.glob("*.jpg")) + list(ASSETS_DIR.glob("*.png"))
    
    if not image_files:
        print("No sample images found in assets directory!")
        return []
    
    for i, img_path in enumerate(image_files, 1):
        size_kb = img_path.stat().st_size / 1024
        print(f"{i}. {img_path.name} ({size_kb:.1f} KB)")
    
    return image_files

def example_1_standalone_usage():
    """Example 1: Using the standalone image describer"""
    print_header("EXAMPLE 1: STANDALONE IMAGE DESCRIPTION")
    
    sample_images = list_sample_images()
    if not sample_images:
        print("No sample images available for testing!")
        return
    
    # Use the first available image
    test_image = sample_images[0]
    print(f"\nUsing image: {test_image.name}")
    
    print("\n1. Basic description:")
    print("Command: ros2 run vlm_inspection image_describer assets/cat_sample.jpg")
    print("(This would run the standalone script with the sample image)")
    
    print("\n2. With different task:")
    print("Command: ros2 run vlm_inspection image_describer assets/cat_sample.jpg --task '<CAPTION>'")
    
    print("\n3. With verbose output:")
    print("Command: ros2 run vlm_inspection image_describer assets/cat_sample.jpg --verbose")
    
    print("\n4. Using different model:")
    print("Command: ros2 run vlm_inspection image_describer assets/cat_sample.jpg --model 'microsoft/Florence-2-base'")

def example_2_ros_node_usage():
    """Example 2: Using the ROS2 node"""
    print_header("EXAMPLE 2: ROS2 NODE USAGE")
    
    print("Step 1: Launch the Florence node")
    print("Command: ros2 launch vlm_inspection florence_launch.py")
    print("")
    print("Step 2: In another terminal, publish an image")
    print("Command: python3 examples/image_publisher.py assets/cat_sample.jpg")
    print("")
    print("Step 3: Listen to descriptions")
    print("Command: ros2 topic echo /florence/description")
    print("")
    print("Alternative: Test with camera")
    print("Command: ros2 run usb_cam usb_cam_node_exe")

def example_3_programmatic_usage():
    """Example 3: Using the Florence service programmatically"""
    print_header("EXAMPLE 3: PROGRAMMATIC USAGE")
    
    print("Here's how to use the Florence service in your own Python code:")
    print("")
    
    example_code = '''
# Import the Florence service
from vlm_inspection.florence_service import FlorenceService

# Initialize the service
florence = FlorenceService("microsoft/Florence-2-large")

# Load the model (this takes time on first run)
if florence.load_model():
    print("Model loaded successfully!")
    
    # Describe an image
    result = florence.describe_image_from_path("path/to/image.jpg")
    
    if "error" not in result:
        print("Description:", result['result'])
    else:
        print("Error:", result['error'])
else:
    print("Failed to load model")
'''
    
    print(example_code)

def example_4_different_tasks():
    """Example 4: Different Florence-2 tasks"""
    print_header("EXAMPLE 4: DIFFERENT FLORENCE-2 TASKS")
    
    tasks = [
        ("<CAPTION>", "Basic image caption"),
        ("<DETAILED_CAPTION>", "Detailed image caption"),
        ("<MORE_DETAILED_CAPTION>", "Very detailed caption (default)"),
        ("<OD>", "Object detection"),
        ("<DENSE_REGION_CAPTION>", "Dense region captioning"),
        ("<OCR>", "Optical character recognition"),
        ("<OCR_WITH_REGION>", "OCR with region information")
    ]
    
    sample_images = list_sample_images()
    if sample_images:
        test_image = sample_images[0].name
        
        print(f"Using sample image: {test_image}")
        print("")
        
        for task, description in tasks:
            print(f"Task: {task}")
            print(f"Description: {description}")
            print(f"Command: ros2 run vlm_inspection image_describer assets/{test_image} --task '{task}'")
            print("")

def run_quick_test():
    """Run a quick test if dependencies are available"""
    print_header("QUICK TEST")
    
    try:
        # Test if we can import the Florence service
        sys.path.insert(0, str(PACKAGE_ROOT))
        from vlm_inspection.florence_service import FlorenceService
        
        # Create service instance
        service = FlorenceService()
        print("✓ Florence service can be imported and initialized")
        
        # Check available tasks
        tasks = service.get_available_tasks()
        print(f"✓ Available tasks: {len(tasks)}")
        
        # List sample images
        sample_images = list_sample_images()
        if sample_images:
            print(f"✓ Found {len(sample_images)} sample images")
        
        print("\n🎉 Package appears to be working correctly!")
        print("📝 Note: Model downloading will happen on first actual use")
        
    except Exception as e:
        print(f"✗ Test failed: {str(e)}")
        print("💡 Make sure to install dependencies: pip install -r requirements.txt")

def show_build_instructions():
    """Show how to build and use the package"""
    print_header("BUILD AND RUN INSTRUCTIONS")
    
    print("1. Install Python dependencies:")
    print("   cd /home/muhayy/vlm_ws/src/vlm_inspection")
    print("   pip install -r requirements.txt")
    print("   # OR run: ./setup.sh")
    print("")
    
    print("2. Build the ROS2 package:")
    print("   cd /home/muhayy/vlm_ws")
    print("   colcon build --packages-select vlm_inspection")
    print("")
    
    print("3. Source the workspace:")
    print("   source install/setup.bash")
    print("")
    
    print("4. Test the installation:")
    print("   python3 src/vlm_inspection/test_installation.py")
    print("")
    
    print("5. Run examples:")
    print("   # Standalone usage:")
    print("   ros2 run vlm_inspection image_describer src/vlm_inspection/assets/cat_sample.jpg")
    print("")
    print("   # ROS2 node (in separate terminals):")
    print("   ros2 launch vlm_inspection florence_launch.py")
    print("   python3 src/vlm_inspection/examples/image_publisher.py src/vlm_inspection/assets/cat_sample.jpg")
    print("   ros2 topic echo /florence/description")

def main():
    """Main function to run all examples"""
    print("🤖 VLM Inspection Package - Usage Examples")
    print("Florence-2 Integration for ROS2")
    
    # Check dependencies first
    deps_ok = check_dependencies()
    
    # Show examples
    example_1_standalone_usage()
    example_2_ros_node_usage()
    example_3_programmatic_usage()
    example_4_different_tasks()
    
    # Show build instructions
    show_build_instructions()
    
    # Run quick test if dependencies are available
    if deps_ok:
        run_quick_test()
    else:
        print_header("NEXT STEPS")
        print("1. Install missing dependencies")
        print("2. Run this script again to see the quick test")
    
    print_header("ADDITIONAL RESOURCES")
    print("📚 README.md - Comprehensive documentation")
    print("⚙️  config/florence_config.yaml - Configuration options")
    print("🧪 examples/ - More example scripts")
    print("📦 assets/ - Sample images for testing")
    
    print(f"\n✨ Sample images available in: {ASSETS_DIR}")
    print("🚀 Ready to start using Florence-2 with ROS2!")

if __name__ == "__main__":
    main()
