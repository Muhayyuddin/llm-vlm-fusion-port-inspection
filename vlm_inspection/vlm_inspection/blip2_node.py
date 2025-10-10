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

from .blip2_service import BLIP2Service

class BLIP2Node(Node):
    """
    ROS2 Node for BLIP2 image description service
    """
    
    def __init__(self):
        super().__init__('blip2_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('model_name', 'Salesforce/blip2-flan-t5-xl')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/blip2/description')
        self.declare_parameter('question_topic', '/blip2/question')
        self.declare_parameter('answer_topic', '/blip2/answer')
        
        # Get parameters
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        question_topic = self.get_parameter('question_topic').get_parameter_value().string_value
        answer_topic = self.get_parameter('answer_topic').get_parameter_value().string_value
        
        self.get_logger().info(f'Initializing BLIP2 Node with model: {model_name}')
        
        # Initialize BLIP2 service
        self.blip2_service = BLIP2Service(model_name)
        
        # Load the model
        self.get_logger().info('Loading BLIP2 model... This may take several minutes.')
        if not self.blip2_service.load_model():
            self.get_logger().error('Failed to load BLIP2 model')
            raise RuntimeError('Failed to load BLIP2 model')
        
        self.get_logger().info('BLIP2 model loaded successfully')
        
        # Store the latest image for question answering
        self.latest_image = None
        
        # Create subscribers
        self.image_subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.question_subscription = self.create_subscription(
            String,
            question_topic,
            self.question_callback,
            10
        )
        
        # Create publishers
        self.description_publisher = self.create_publisher(String, output_topic, 10)
        self.answer_publisher = self.create_publisher(String, answer_topic, 10)
        
        self.get_logger().info(f'BLIP2 Node initialized')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing descriptions to: {output_topic}')
        self.get_logger().info(f'Subscribing to questions on: {question_topic}')
        self.get_logger().info(f'Publishing answers to: {answer_topic}')
    
    def image_callback(self, msg: Image):
        """
        Callback function for incoming images
        
        Args:
            msg: ROS2 Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to RGB for PIL
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            pil_image = PILImage.fromarray(rgb_image)
            
            # Store the latest image for question answering
            self.latest_image = pil_image
            
            # Generate description using BLIP2
            result = self.blip2_service.describe_image(pil_image)
            
            if 'error' in result:
                self.get_logger().error(f'BLIP2 description failed: {result["error"]}')
                return
            
            # Create and publish description message
            description_msg = String()
            description_msg.data = result['result']
            
            self.description_publisher.publish(description_msg)
            
            self.get_logger().info(f'Published description: {result["result"][:100]}...')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def question_callback(self, msg: String):
        """
        Callback function for incoming questions
        
        Args:
            msg: ROS2 String message containing the question
        """
        if self.latest_image is None:
            self.get_logger().warning('No image available for question answering')
            return
        
        try:
            question = msg.data
            self.get_logger().info(f'Received question: {question}')
            
            # Answer the question using BLIP2
            result = self.blip2_service.answer_question(self.latest_image, question)
            
            if 'error' in result:
                self.get_logger().error(f'BLIP2 question answering failed: {result["error"]}')
                return
            
            # Create and publish answer message
            answer_msg = String()
            answer_msg.data = result['result']
            
            self.answer_publisher.publish(answer_msg)
            
            self.get_logger().info(f'Published answer: {result["result"]}')
            
        except Exception as e:
            self.get_logger().error(f'Error answering question: {str(e)}')

def main(args=None):
    """
    Main function to run the BLIP2 node
    """
    rclpy.init(args=args)
    
    try:
        blip2_node = BLIP2Node()
        rclpy.spin(blip2_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running BLIP2 node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
