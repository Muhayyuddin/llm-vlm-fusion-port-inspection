#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path
import logging

from .florence_service import FlorenceService

def main():
    """
    Standalone image description script using Florence-2
    """
    # Setup argument parser
    parser = argparse.ArgumentParser(description='Describe images using Florence-2')
    parser.add_argument('image_path', type=str, help='Path to the image file')
    parser.add_argument('--task', type=str, default='<MORE_DETAILED_CAPTION>',
                       help='Florence-2 task type (default: <MORE_DETAILED_CAPTION>)')
    parser.add_argument('--model', type=str, default='microsoft/Florence-2-large',
                       help='Florence-2 model name (default: microsoft/Florence-2-large)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    
    args = parser.parse_args()
    
    # Setup logging
    log_level = logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=log_level, format='%(levelname)s: %(message)s')
    
    # Validate input file
    image_path = Path(args.image_path)
    if not image_path.exists():
        print(f"Error: Image file '{args.image_path}' does not exist.")
        sys.exit(1)
    
    if not image_path.is_file():
        print(f"Error: '{args.image_path}' is not a file.")
        sys.exit(1)
    
    # Initialize Florence service
    florence_service = FlorenceService(model_name=args.model)
    
    print(f"Loading Florence-2 model: {args.model}")
    if not florence_service.load_model():
        print("Error: Failed to load Florence-2 model.")
        sys.exit(1)
    
    print(f"Analyzing image: {args.image_path}")
    print(f"Task: {args.task}")
    print("-" * 50)
    
    # Describe the image
    result = florence_service.describe_image_from_path(str(image_path), args.task)
    
    if "error" in result:
        print(f"Error: {result['error']}")
        sys.exit(1)
    
    # Display results
    print("Description:")
    print(result['result'])
    print(f"\nImage size: {result['image_size']}")
    
    # Show available tasks if verbose
    if args.verbose:
        print("\nAvailable tasks:")
        for task in florence_service.get_available_tasks():
            print(f"  - {task}")

if __name__ == "__main__":
    main()
