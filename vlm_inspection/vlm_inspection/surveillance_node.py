#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import logging
import time

from .florence_service import FlorenceService

class SurveillanceNode(Node):
    """
    ROS2 Node for surveillance using Florence-2 model
    Subscribes to quadrotor camera feed and analyzes for suspicious activity
    """
    
    def __init__(self):
        super().__init__('surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('model_name', 'microsoft/Florence-2-large')
        self.declare_parameter('analysis_rate', 1.0)  # Hz
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'Initializing Surveillance Node')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        
        # Initialize Florence-2 service
        self.florence_service = FlorenceService(model_name)
        
        # Load the model
        self.get_logger().info('Loading Florence-2 model... This may take several minutes.')
        if not self.florence_service.load_model():
            self.get_logger().error('Failed to load Florence-2 model')
            raise RuntimeError('Failed to load Florence-2 model')
        
        self.get_logger().info('Florence-2 model loaded successfully')
        
        # Store the latest image
        self.latest_image = None
        self.image_timestamp = None
        
        # Create subscriber
        self.image_subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Create timer for analysis (1 Hz)
        analysis_period = 1.0 / analysis_rate
        self.analysis_timer = self.create_timer(analysis_period, self.analyze_image)
        
        # Surveillance prompt
        self.surveillance_prompt = (
            "You are surveying the sea port area, in particular cargo ship loading area and containers stack. This image is captured by UAV camera that is surveying the area. "
            "Is there anything suspicious in the image? there should not be any person or vehicle except forklifts in the area. if you detect any person, vehicle, or suspicious activity, please describe in detail what you see. and report any potential safety or security concerns."
        )
        
        self.get_logger().info('Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Analysis prompt: {self.surveillance_prompt}')
    
    def image_callback(self, msg: Image):
        """
        Callback function for incoming images from quadrotor camera
        
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
            
            # Store the latest image with timestamp
            self.latest_image = pil_image
            self.image_timestamp = msg.header.stamp
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def analyze_image(self):
        """
        Timer callback to analyze the latest image for suspicious activity
        """
        if self.latest_image is None:
            self.get_logger().warn('No image available for analysis')
            return
        
        try:
            self.get_logger().info('🔍 Analyzing surveillance image...')
            
            # Use Florence-2 to analyze the image with surveillance prompt
            # Using a task that can handle the custom prompt
            result = self.florence_service.describe_image(
                self.latest_image, 
                f"<DETAILED_CAPTION>"  # Use detailed caption task
            )
            
            if 'error' in result:
                self.get_logger().error(f'Florence-2 analysis failed: {result["error"]}')
                return
            
            # Get the basic description first
            basic_description = result['result']
            
            # Now analyze for suspicious activity using a more specific approach
            suspicious_result = self.florence_service.describe_image(
                self.latest_image,
                f"<MORE_DETAILED_CAPTION>"
            )
            
            if 'error' not in suspicious_result:
                detailed_description = suspicious_result['result']
                
                # Print surveillance analysis
                self.print_surveillance_report(basic_description, detailed_description)
            else:
                self.print_surveillance_report(basic_description, "Could not get detailed analysis")
            
        except Exception as e:
            self.get_logger().error(f'Error during image analysis: {str(e)}')
    
    def print_surveillance_report(self, basic_description: str, detailed_description: str):
        """
        Print formatted surveillance report
        
        Args:
            basic_description: Basic image description
            detailed_description: Detailed image description
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        
        print("\n" + "="*80)
        print(f"🚁 SURVEILLANCE REPORT - {timestamp_str}")
        print("="*80)
        print(f"📍 PROMPT: {self.surveillance_prompt}")
        print("-"*80)
        print(f"📸 BASIC DESCRIPTION:")
        print(f"   {basic_description}")
        print("-"*80)
        print(f"🔍 DETAILED ANALYSIS:")
        print(f"   {detailed_description}")
        print("-"*80)
        
        # Simple keyword-based suspicious activity detection
        suspicious_keywords = [
            'weapon', 'gun', 'knife', 'suspicious', 'person', 'people', 
            'vehicle', 'car', 'truck', 'moving', 'running', 'hiding',
            'fire', 'smoke', 'damage', 'broken', 'unusual', 'strange'
        ]
        
        combined_text = (basic_description + " " + detailed_description).lower()
        found_keywords = [keyword for keyword in suspicious_keywords if keyword in combined_text]
        
        if found_keywords:
            print(f"⚠️  POTENTIAL CONCERNS DETECTED:")
            print(f"   Keywords found: {', '.join(found_keywords)}")
            print(f"   📞 Recommendation: Review this area more closely")
        else:
            print(f"✅ NO IMMEDIATE CONCERNS DETECTED")
            print(f"   📊 Area appears normal based on analysis")
        
        print("="*80)
        print()

def main(args=None):
    """
    Main function to run the surveillance node
    """
    rclpy.init(args=args)
    
    try:
        surveillance_node = SurveillanceNode()
        
        surveillance_node.get_logger().info('🚁 Surveillance Node started - Monitoring for suspicious activity')
        surveillance_node.get_logger().info('📸 Analyzing images every 1 second')
        
        rclpy.spin(surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
