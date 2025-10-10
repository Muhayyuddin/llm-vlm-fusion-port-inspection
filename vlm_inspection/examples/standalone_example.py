#!/usr/bin/env python3
"""
Simple example demonstrating Florence-2 usage outside of ROS2
This is equivalent to your original script but using the Florence service
"""

import sys
from pathlib import Path

# Add the package to Python path (for standalone testing)
package_path = Path(__file__).parent.parent / "vlm_inspection"
sys.path.insert(0, str(package_path.parent))

from vlm_inspection.florence_service import FlorenceService

def main():
    """
    Example usage of Florence-2 service
    """
    # Initialize the Florence service
    print("Initializing Florence-2 service...")
    florence_service = FlorenceService(model_name="microsoft/Florence-2-large")
    
    # Load the model
    print("Loading Florence-2 model (this may take a while on first run)...")
    if not florence_service.load_model():
        print("Failed to load model!")
        return 1
    
    print("Model loaded successfully!")
    
    # Example image path (you need to provide an actual image)
    image_path = input("Enter path to your image file: ").strip()
    
    if not Path(image_path).exists():
        print(f"Image file '{image_path}' does not exist!")
        return 1
    
    # Generate description
    print("Generating description...")
    result = florence_service.describe_image_from_path(
        image_path, 
        task="<MORE_DETAILED_CAPTION>"
    )
    
    if "error" in result:
        print(f"Error: {result['error']}")
        return 1
    
    # Display result
    print("\nDescription:")
    print("-" * 50)
    print(result['result'])
    print("-" * 50)
    print(f"Image size: {result['image_size']}")
    
    # Show available tasks
    print(f"\nTask used: {result['task']}")
    print("\nOther available tasks:")
    for task in florence_service.get_available_tasks():
        print(f"  - {task}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
