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

from .nanovlm_service import NanoVLMService

class NanoVLMSurveillanceNode(Node):
    """
    ROS2 Node for surveillance using nanoVLM model (~222M parameters)
    Ultra-compact and efficient surveillance for captioning & VQA
    Subscribes to quadrotor camera feed and analyzes for suspicious activity
    """
    
    def __init__(self):
        super().__init__('nanovlm_surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('model_name', 'microsoft/nanoVLM')
        self.declare_parameter('analysis_rate', 2.0)  # Higher rate due to efficiency
        self.declare_parameter('enable_vqa', True)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        enable_vqa = self.get_parameter('enable_vqa').get_parameter_value().bool_value
        
        self.get_logger().info(f'Initializing Compact VLM Surveillance Node (using Florence-2-base)')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        self.get_logger().info(f'VQA enabled: {enable_vqa}')
        
        # Initialize compact VLM service (using Florence-2-base)
        self.nanovlm_service = NanoVLMService(model_name)
        self.enable_vqa = enable_vqa
        
        # Load the model
        self.get_logger().info('Loading compact VLM model (Florence-2-base)...')
        if not self.nanovlm_service.load_model():
            self.get_logger().error('Failed to load compact VLM model')
            raise RuntimeError('Failed to load compact VLM model')
        
        self.get_logger().info('Compact VLM model loaded successfully')
        
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
        
        # Create timer for analysis
        analysis_period = 1.0 / analysis_rate
        self.analysis_timer = self.create_timer(analysis_period, self.analyze_image)
        
        # Surveillance prompts optimized for nanoVLM
        self.surveillance_prompt = (
            "Analyze this port surveillance image. Look for: unauthorized people, "
            "suspicious vehicles, safety hazards, or unusual activities. "
            "Only trucks and forklifts are allowed. Report any concerns."
        )
        
        self.vqa_questions = [
            "Are there any people visible in this image?",
            "Are there any unauthorized vehicles?", 
            "Do you see any safety hazards?",
            "Is there anything unusual or suspicious?"
        ]
        
        self.get_logger().info('nanoVLM Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Model info: {self.nanovlm_service.get_model_info()}')
    
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
            self.get_logger().info('🔍 Analyzing surveillance image with compact VLM...')
            
            # Get basic surveillance analysis
            surveillance_result = self.nanovlm_service.detect_surveillance_anomalies(self.latest_image)
            
            if 'error' in surveillance_result:
                self.get_logger().error(f'Compact VLM analysis failed: {surveillance_result["error"]}')
                return
            
            surveillance_analysis = surveillance_result['result']
            
            # Perform VQA if enabled (using Florence-2 style captions)
            vqa_results = {}
            if self.enable_vqa:
                for question in self.vqa_questions:
                    # Use basic description for VQA-style questions
                    vqa_result = self.nanovlm_service.describe_image(self.latest_image, "<DETAILED_CAPTION>")
                    if 'error' not in vqa_result:
                        vqa_results[question] = vqa_result['result']
            
            # Print comprehensive surveillance report
            self.print_surveillance_report(surveillance_analysis, vqa_results)
            
        except Exception as e:
            self.get_logger().error(f'Error during image analysis: {str(e)}')
    
    def print_surveillance_report(self, surveillance_analysis: str, vqa_results: dict):
        """
        Print formatted surveillance report
        
        Args:
            surveillance_analysis: Main surveillance analysis
            vqa_results: VQA question-answer pairs
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        
        print("\n" + "="*80)
        print(f"🔬 Compact VLM SURVEILLANCE REPORT - {timestamp_str}")
        print("="*80)
        print(f"📍 SURVEILLANCE ANALYSIS:")
        print(f"   {surveillance_analysis}")
        print("-"*80)
        
        # Display VQA results if available
        if vqa_results:
            print(f"❓ VISUAL Q&A ANALYSIS:")
            for question, answer in vqa_results.items():
                print(f"   Q: {question}")
                print(f"   A: {answer}")
                print("-"*40)
        
        # Enhanced keyword-based analysis
        suspicious_keywords = [
            'person', 'people', 'individual', 'human', 'man', 'woman',
            'unauthorized', 'suspicious', 'unusual', 'strange', 'unexpected',
            'weapon', 'gun', 'knife', 'dangerous',
            'car', 'sedan', 'motorcycle', 'bicycle', 'unauthorized vehicle',
            'fire', 'smoke', 'explosion', 'leak', 'spill',
            'damage', 'broken', 'fallen', 'collapsed'
        ]
        
        safety_keywords = [
            'hazard', 'danger', 'unsafe', 'risk', 'warning',
            'emergency', 'accident', 'incident'
        ]
        
        combined_text = (surveillance_analysis + " " + " ".join(vqa_results.values())).lower()
        found_suspicious = [keyword for keyword in suspicious_keywords if keyword in combined_text]
        found_safety = [keyword for keyword in safety_keywords if keyword in combined_text]
        
        # Determine threat level
        if found_suspicious or found_safety:
            threat_level = "HIGH" if len(found_suspicious) > 2 or len(found_safety) > 1 else "MEDIUM"
            print(f"⚠️  THREAT LEVEL: {threat_level}")
            
            if found_suspicious:
                print(f"🚨 SUSPICIOUS INDICATORS: {', '.join(found_suspicious)}")
            if found_safety:
                print(f"⚠️  SAFETY CONCERNS: {', '.join(found_safety)}")
                
            print(f"📞 RECOMMENDATION: Immediate review and potential response required")
        else:
            print(f"✅ AREA STATUS: NORMAL")
            print(f"📊 No immediate concerns detected")
        
        print(f"🔬 Model: Compact VLM (Florence-2-base) - Efficient analysis")
        print("="*80)
        print()

def main(args=None):
    """
    Main function to run the nanoVLM surveillance node
    """
    rclpy.init(args=args)
    
    try:
        surveillance_node = NanoVLMSurveillanceNode()
        
        surveillance_node.get_logger().info('🔬 Compact VLM Surveillance Node started - Efficient monitoring')
        surveillance_node.get_logger().info('📸 Analyzing images with compact VLM model')
        
        rclpy.spin(surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Compact VLM surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running compact VLM surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
