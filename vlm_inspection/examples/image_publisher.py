#!/usr/bin/env python3
"""
Example ROS2 node that publishes images for testing the Florence node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
import argparse

class ImagePublisher(Node):
    """Simple image publisher for testing"""
    
    def __init__(self, image_path: str, rate: float = 1.0):
        super().__init__('image_publisher')
        
        self.image_path = Path(image_path)
        if not self.image_path.exists():
            self.get_logger().error(f"Image file '{image_path}' does not exist!")
            raise FileNotFoundError(f"Image file '{image_path}' does not exist!")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Subscriber to listen for descriptions
        self.description_subscriber = self.create_subscription(
            String,
            'florence/description',
            self.description_callback,
            10
        )
        
        # Timer for publishing images
        self.timer = self.create_timer(1.0 / rate, self.publish_image)
        
        # Load the image
        self.cv_image = cv2.imread(str(self.image_path))
        if self.cv_image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            raise ValueError(f"Failed to load image: {image_path}")
        
        # Convert BGR to RGB for ROS
        self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        
        self.get_logger().info(f"Publishing image: {image_path}")
        self.get_logger().info(f"Image shape: {self.cv_image.shape}")
    
    def publish_image(self):
        """Publish the image"""
        try:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # Publish the image
            self.image_publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")
    
    def description_callback(self, msg: String):
        """Callback for receiving descriptions"""
        self.get_logger().info(f"Received description: {msg.data}")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Publish test images for Florence node')
    parser.add_argument('image_path', type=str, help='Path to image file')
    parser.add_argument('--rate', type=float, default=0.5, 
                       help='Publishing rate in Hz (default: 0.5)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        image_publisher = ImagePublisher(args.image_path, args.rate)
        
        print(f"Publishing image: {args.image_path}")
        print(f"Rate: {args.rate} Hz")
        print("Listening for descriptions on /florence/description")
        print("Press Ctrl+C to stop...")
        
        rclpy.spin(image_publisher)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
