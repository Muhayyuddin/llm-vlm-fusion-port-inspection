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

from .blip2_service import BLIP2Service

class BLIP2SurveillanceNode(Node):
    """
    ROS2 Node for surveillance using BLIP2 model
    Subscribes to quadrotor camera feed and analyzes for suspicious activity
    """
    
    def __init__(self):
        super().__init__('blip2_surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('model_name', 'Salesforce/blip2-opt-2.7b')  # Memory-efficient model
        self.declare_parameter('analysis_rate', 1.0)  # Hz
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'Initializing BLIP2 Surveillance Node')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Model: {model_name}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        
        # Initialize BLIP2 service
        self.blip2_service = BLIP2Service(model_name)
        
        # Load the model
        self.get_logger().info('Loading BLIP2 model... This may take several minutes.')
        if not self.blip2_service.load_model():
            self.get_logger().error('Failed to load BLIP2 model')
            raise RuntimeError('Failed to load BLIP2 model')
        
        self.get_logger().info('BLIP2 model loaded successfully')
        
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
        
        # Surveillance prompts for BLIP2
        self.surveillance_questions = [
            "I am surveying the area this is image is captured by my camera is there anything suspicious in the image",
            "What do you see in this surveillance image? Are there any people, vehicles, or unusual activities?",
            "Describe any potential security concerns or suspicious activities in this image"
        ]
        
        self.get_logger().info('BLIP2 Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Primary surveillance question: {self.surveillance_questions[0]}')
    
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
            self.get_logger().info('🔍 Analyzing surveillance image with BLIP2...')
            
            # Get basic image description first
            basic_description = self.blip2_service.describe_image(self.latest_image)
            
            if 'error' in basic_description:
                self.get_logger().error(f'BLIP2 description failed: {basic_description["error"]}')
                return
            
            # Ask surveillance-specific question
            surveillance_analysis = self.blip2_service.answer_question(
                self.latest_image, 
                self.surveillance_questions[0]
            )
            
            if 'error' in surveillance_analysis:
                surveillance_answer = "Could not analyze for suspicious activity"
            else:
                surveillance_answer = surveillance_analysis['answer']
            
            # Get additional security assessment
            security_analysis = self.blip2_service.answer_question(
                self.latest_image,
                self.surveillance_questions[1]
            )
            
            if 'error' not in security_analysis:
                security_answer = security_analysis['answer']
            else:
                security_answer = "Security assessment unavailable"
            
            # Print comprehensive surveillance report
            self.print_surveillance_report(
                basic_description['description'], 
                surveillance_answer, 
                security_answer
            )
            
        except Exception as e:
            self.get_logger().error(f'Error during image analysis: {str(e)}')
    
    def print_surveillance_report(self, basic_description: str, surveillance_answer: str, security_answer: str):
        """
        Print formatted surveillance report using BLIP2 analysis
        
        Args:
            basic_description: Basic image description
            surveillance_answer: Answer to surveillance question
            security_answer: Security assessment answer
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        
        print("\n" + "="*80)
        print(f"🚁 BLIP2 SURVEILLANCE REPORT - {timestamp_str}")
        print("="*80)
        print(f"📍 PRIMARY QUESTION: {self.surveillance_questions[0]}")
        print("-"*80)
        print(f"📸 BASIC DESCRIPTION:")
        print(f"   {basic_description}")
        print("-"*80)
        print(f"🔍 SURVEILLANCE ANALYSIS:")
        print(f"   {surveillance_answer}")
        print("-"*80)
        print(f"🛡️  SECURITY ASSESSMENT:")
        print(f"   {security_answer}")
        print("-"*80)
        
        # Enhanced keyword-based suspicious activity detection
        suspicious_keywords = {
            'weapons': ['weapon', 'gun', 'knife', 'rifle', 'pistol', 'firearm', 'armed'],
            'people': ['person', 'people', 'individual', 'man', 'woman', 'child', 'crowd'],
            'suspicious_behavior': ['suspicious', 'running', 'hiding', 'sneaking', 'lurking', 'trespassing'],
            'vehicles': ['vehicle', 'car', 'truck', 'van', 'motorcycle', 'bike', 'moving'],
            'threats': ['fire', 'smoke', 'explosion', 'damage', 'broken', 'destroyed'],
            'unusual': ['unusual', 'strange', 'odd', 'unexpected', 'abnormal', 'concerning']
        }
        
        # Combine all text for analysis
        combined_text = f"{basic_description} {surveillance_answer} {security_answer}".lower()
        
        # Categorize detected keywords
        detected_categories = {}
        total_keywords = []
        
        for category, keywords in suspicious_keywords.items():
            found_in_category = [keyword for keyword in keywords if keyword in combined_text]
            if found_in_category:
                detected_categories[category] = found_in_category
                total_keywords.extend(found_in_category)
        
        # Generate threat assessment
        if total_keywords:
            threat_level = self.assess_threat_level(detected_categories)
            print(f"⚠️  POTENTIAL CONCERNS DETECTED - {threat_level} PRIORITY:")
            
            for category, keywords in detected_categories.items():
                print(f"   📋 {category.upper()}: {', '.join(keywords)}")
            
            # Provide recommendations based on detected categories
            recommendations = self.generate_recommendations(detected_categories)
            print(f"   📞 RECOMMENDATIONS:")
            for rec in recommendations:
                print(f"     • {rec}")
        else:
            print(f"✅ NO IMMEDIATE CONCERNS DETECTED")
            print(f"   📊 Area appears normal based on BLIP2 analysis")
        
        print("="*80)
        print()
    
    def assess_threat_level(self, detected_categories: dict) -> str:
        """
        Assess threat level based on detected keyword categories
        
        Args:
            detected_categories: Dictionary of detected keyword categories
            
        Returns:
            Threat level string
        """
        high_priority = ['weapons', 'threats']
        medium_priority = ['suspicious_behavior', 'unusual']
        low_priority = ['people', 'vehicles']
        
        if any(cat in detected_categories for cat in high_priority):
            return "HIGH"
        elif any(cat in detected_categories for cat in medium_priority):
            return "MEDIUM"
        elif any(cat in detected_categories for cat in low_priority):
            return "LOW"
        else:
            return "INFORMATIONAL"
    
    def generate_recommendations(self, detected_categories: dict) -> list:
        """
        Generate specific recommendations based on detected categories
        
        Args:
            detected_categories: Dictionary of detected keyword categories
            
        Returns:
            List of recommendation strings
        """
        recommendations = []
        
        if 'weapons' in detected_categories:
            recommendations.append("IMMEDIATE: Alert security personnel about potential weapons")
        
        if 'threats' in detected_categories:
            recommendations.append("URGENT: Investigate fire/damage reports immediately")
        
        if 'suspicious_behavior' in detected_categories:
            recommendations.append("Monitor area closely for unusual behavior patterns")
        
        if 'people' in detected_categories:
            recommendations.append("Track personnel movement in monitored area")
        
        if 'vehicles' in detected_categories:
            recommendations.append("Log vehicle activity and verify authorization")
        
        if 'unusual' in detected_categories:
            recommendations.append("Document unusual activity for further analysis")
        
        if not recommendations:
            recommendations.append("Continue routine monitoring")
        
        return recommendations

def main(args=None):
    """
    Main function to run the BLIP2 surveillance node
    """
    rclpy.init(args=args)
    
    try:
        blip2_surveillance_node = BLIP2SurveillanceNode()
        
        blip2_surveillance_node.get_logger().info('🚁 BLIP2 Surveillance Node started - Monitoring for suspicious activity')
        blip2_surveillance_node.get_logger().info('📸 Analyzing images every 1 second with BLIP2')
        
        rclpy.spin(blip2_surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 BLIP2 Surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running BLIP2 surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
