#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import logging
import threading

from .florence_service import FlorenceService

class FlorenceNode(Node):
    """
    ROS2 Node for Florence-2 image description service
    """
    
    def __init__(self):
        super().__init__('florence_node')
        
        # Setup logging
        self.get_logger().info("Initializing Florence Node")
        
        # Initialize CV Bridge for ROS Image conversion
        self.bridge = CvBridge()
        
        # Initialize Florence service
        self.florence_service = FlorenceService()
        
        # Parameters
        self.declare_parameter('model_name', 'microsoft/Florence-2-large')
        self.declare_parameter('default_task', '<MORE_DETAILED_CAPTION>')
        self.declare_parameter('auto_load_model', True)
        
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.default_task = self.get_parameter('default_task').get_parameter_value().string_value
        auto_load = self.get_parameter('auto_load_model').get_parameter_value().bool_value
        
        # Update Florence service with parameter
        self.florence_service.model_name = model_name
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10
        )
        
        # Publishers
        self.description_publisher = self.create_publisher(
            String,
            'image_description',
            10
        )
        
        # Service for custom task requests (you can extend this later)
        # For now, we'll use topic-based communication
        
        # Load model in background if requested
        if auto_load:
            self.get_logger().info("Loading Florence-2 model in background...")
            self.load_model_thread = threading.Thread(target=self._load_model_background)
            self.load_model_thread.daemon = True
            self.load_model_thread.start()
        
        self.get_logger().info("Florence Node initialized successfully")
    
    def _load_model_background(self):
        """Load the model in a background thread"""
        success = self.florence_service.load_model()
        if success:
            self.get_logger().info("Florence-2 model loaded successfully in background")
        else:
            self.get_logger().error("Failed to load Florence-2 model")
    
    def image_callback(self, msg: Image):
        """
        Callback for incoming image messages
        
        Args:
            msg: ROS Image message
        """
        if not self.florence_service.is_model_loaded():
            self.get_logger().warn("Florence-2 model not loaded yet, skipping image...")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Convert OpenCV image to PIL Image
            pil_image = PILImage.fromarray(cv_image)
            
            # Get description from Florence service
            result = self.florence_service.describe_image(pil_image, self.default_task)
            
            if "error" in result:
                self.get_logger().error(f"Florence service error: {result['error']}")
                return
            
            # Publish the description
            description_msg = String()
            description_msg.data = str(result['result'])
            self.description_publisher.publish(description_msg)
            
            self.get_logger().info(f"Published image description: {description_msg.data}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def load_model_sync(self):
        """Synchronously load the model"""
        return self.florence_service.load_model()

def main(args=None):
    """Main entry point for the Florence node"""
    rclpy.init(args=args)
    
    try:
        florence_node = FlorenceNode()
        
        # Spin the node
        rclpy.spin(florence_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"Error in Florence node: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
