#!/usr/bin/env python3
"""
Image publisher for testing VLM nodes
Publishes sample images to test Florence-2 and BLIP2 nodes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from pathlib import Path
import time
import argparse

class ImagePublisher(Node):
    """
    ROS2 node that publishes images for VLM testing
    """
    
    def __init__(self, image_path: str, publish_rate: float = 1.0):
        super().__init__('image_publisher')
        
        self.bridge = CvBridge()
        self.image_path = Path(image_path)
        
        # Create publishers
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.question_publisher = self.create_publisher(String, '/blip2/question', 10)
        
        # Create subscribers to monitor responses
        self.florence_sub = self.create_subscription(
            String, '/florence/description', self.florence_callback, 10
        )
        self.blip2_sub = self.create_subscription(
            String, '/blip2/description', self.blip2_callback, 10
        )
        self.blip2_answer_sub = self.create_subscription(
            String, '/blip2/answer', self.blip2_answer_callback, 10
        )
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)
        
        # Load image
        if not self.image_path.exists():
            self.get_logger().error(f'Image not found: {self.image_path}')
            raise FileNotFoundError(f'Image not found: {self.image_path}')
        
        self.cv_image = cv2.imread(str(self.image_path))
        if self.cv_image is None:
            self.get_logger().error(f'Failed to load image: {self.image_path}')
            raise ValueError(f'Failed to load image: {self.image_path}')
        
        self.get_logger().info(f'Publishing image: {self.image_path}')
        self.get_logger().info(f'Image size: {self.cv_image.shape}')
        
        # Counter for questions
        self.question_count = 0
        self.questions = [
            "What do you see in this image?",
            "What colors are visible?",
            "How many objects are there?",
            "Describe the main subject.",
            "What is happening in this scene?"
        ]
        
        # Timer for asking questions
        self.question_timer = self.create_timer(5.0, self.ask_question)
    
    def publish_image(self):
        """Publish the loaded image"""
        try:
            # Convert OpenCV image to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera'
            
            # Publish image
            self.image_publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def ask_question(self):
        """Ask a question about the image"""
        if self.question_count < len(self.questions):
            question = self.questions[self.question_count]
            msg = String()
            msg.data = question
            
            self.question_publisher.publish(msg)
            self.get_logger().info(f'Asked question: {question}')
            
            self.question_count += 1
        else:
            # Reset question counter
            self.question_count = 0
    
    def florence_callback(self, msg: String):
        """Handle Florence-2 descriptions"""
        self.get_logger().info(f'Florence-2: {msg.data}')
    
    def blip2_callback(self, msg: String):
        """Handle BLIP2 descriptions"""
        self.get_logger().info(f'BLIP2 Description: {msg.data}')
    
    def blip2_answer_callback(self, msg: String):
        """Handle BLIP2 answers"""
        self.get_logger().info(f'BLIP2 Answer: {msg.data}')

def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='Publish images for VLM testing')
    parser.add_argument('image_path', help='Path to image file')
    parser.add_argument('--rate', type=float, default=1.0, 
                       help='Publishing rate in Hz (default: 1.0)')
    
    # Parse command line arguments
    if args is None:
        import sys
        args = sys.argv[1:]
    
    parsed_args = parser.parse_args(args)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run node
        node = ImagePublisher(parsed_args.image_path, parsed_args.rate)
        
        print(f"📸 Publishing image: {parsed_args.image_path}")
        print("🔍 Listening for VLM responses...")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n👋 Stopping image publisher")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
