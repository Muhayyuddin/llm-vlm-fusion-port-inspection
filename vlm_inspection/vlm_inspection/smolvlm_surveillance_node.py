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

from .smolvlm_service import SmolVLMService

class SmolVLMSurveillanceNode(Node):
    """
    ROS2 Node for surveillance using SmolVLM model (256M/500M parameters)
    Minimal footprint with strong multimodal capability
    Subscribes to quadrotor camera feed and analyzes for suspicious activity
    """
    
    def __init__(self):
        super().__init__('smolvlm_surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('model_name', 'HuggingFaceTB/SmolVLM-256M-Instruct')
        self.declare_parameter('analysis_rate', 1.5)  # Good balance for quality/speed
        self.declare_parameter('enable_compliance_check', True)
        self.declare_parameter('enable_detailed_vqa', True)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        enable_compliance = self.get_parameter('enable_compliance_check').get_parameter_value().bool_value
        enable_detailed_vqa = self.get_parameter('enable_detailed_vqa').get_parameter_value().bool_value
        
        # Determine model size
        model_size = "256M" if "256M" in model_name else "500M"
        
        self.get_logger().info(f'Initializing SmolVLM-{model_size} Surveillance Node')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        self.get_logger().info(f'Compliance checking: {enable_compliance}')
        self.get_logger().info(f'Detailed VQA: {enable_detailed_vqa}')
        
        # Initialize SmolVLM service
        self.smolvlm_service = SmolVLMService(model_name)
        self.enable_compliance = enable_compliance
        self.enable_detailed_vqa = enable_detailed_vqa
        
        # Load the model
        self.get_logger().info(f'Loading SmolVLM-{model_size} model...')
        if not self.smolvlm_service.load_model():
            self.get_logger().error('Failed to load SmolVLM model')
            raise RuntimeError('Failed to load SmolVLM model')
        
        self.get_logger().info('SmolVLM model loaded successfully')
        
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
        
        # Enhanced surveillance configuration
        self.surveillance_prompt = (
            "You are analyzing a surveillance image from a maritime port facility. "
            "Conduct a comprehensive security assessment covering:\n"
            "1. Personnel identification (authorized vs unauthorized)\n"
            "2. Vehicle compliance (trucks/forklifts allowed, others restricted)\n" 
            "3. Safety hazards and environmental risks\n"
            "4. Security violations or suspicious activities\n"
            "5. Operational compliance with port regulations\n"
            "Provide detailed observations with confidence levels."
        )
        
        self.detailed_vqa_questions = [
            "How many people are visible in this image and what are they doing?",
            "What types of vehicles are present and are they appropriate for this area?",
            "Are there any safety hazards or environmental concerns visible?",
            "Do you observe any security violations or unauthorized activities?",
            "What is the overall operational status of this port area?",
            "Are there any objects or equipment that appear damaged or misplaced?"
        ]
        
        self.maritime_regulations = (
            "Maritime Port Safety Regulations:\n"
            "- Only authorized personnel with proper safety equipment allowed\n"
            "- Vehicles limited to operational trucks and forklifts\n" 
            "- No smoking or open flames near cargo areas\n"
            "- All personnel must wear high-visibility safety vests\n"
            "- Container stacking must follow height and stability guidelines\n"
            "- Clear emergency exit paths must be maintained"
        )
        
        self.get_logger().info('SmolVLM Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Model info: {self.smolvlm_service.get_model_info()}')
    
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
            self.get_logger().info('🔍 Analyzing surveillance image with SmolVLM...')
            
            # Get main surveillance analysis
            surveillance_result = self.smolvlm_service.detect_surveillance_anomalies(self.latest_image)
            
            if 'error' in surveillance_result:
                self.get_logger().error(f'SmolVLM analysis failed: {surveillance_result["error"]}')
                return
            
            surveillance_analysis = surveillance_result['result']
            
            # Perform detailed VQA if enabled
            vqa_results = {}
            if self.enable_detailed_vqa:
                for question in self.detailed_vqa_questions:
                    vqa_result = self.smolvlm_service.answer_question(self.latest_image, question)
                    if 'error' not in vqa_result:
                        vqa_results[question] = vqa_result['result']
            
            # Perform compliance check if enabled
            compliance_analysis = ""
            if self.enable_compliance:
                compliance_result = self.smolvlm_service.analyze_compliance(
                    self.latest_image, 
                    self.maritime_regulations
                )
                if 'error' not in compliance_result:
                    compliance_analysis = compliance_result['result']
            
            # Generate comprehensive surveillance report
            self.print_surveillance_report(surveillance_analysis, vqa_results, compliance_analysis)
            
        except Exception as e:
            self.get_logger().error(f'Error during image analysis: {str(e)}')
    
    def print_surveillance_report(self, surveillance_analysis: str, vqa_results: dict, compliance_analysis: str):
        """
        Print comprehensive surveillance report
        
        Args:
            surveillance_analysis: Main surveillance analysis
            vqa_results: Detailed VQA question-answer pairs
            compliance_analysis: Regulatory compliance assessment
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        model_info = self.smolvlm_service.get_model_info()
        
        print("\n" + "="*80)
        print(f"🧠 SmolVLM SURVEILLANCE REPORT ({model_info['parameters']}) - {timestamp_str}")
        print("="*80)
        print(f"🔍 COMPREHENSIVE SURVEILLANCE ANALYSIS:")
        print(f"   {surveillance_analysis}")
        print("-"*80)
        
        # Display detailed VQA results
        if vqa_results:
            print(f"❓ DETAILED VISUAL Q&A ASSESSMENT:")
            for i, (question, answer) in enumerate(vqa_results.items(), 1):
                print(f"   {i}. Q: {question}")
                print(f"      A: {answer}")
                print("-"*40)
        
        # Display compliance analysis
        if compliance_analysis:
            print(f"⚖️  REGULATORY COMPLIANCE ASSESSMENT:")
            print(f"   {compliance_analysis}")
            print("-"*80)
        
        # Advanced threat assessment
        all_text = " ".join([surveillance_analysis] + list(vqa_results.values()) + [compliance_analysis])
        
        # Security indicators
        security_indicators = {
            'critical': ['weapon', 'gun', 'knife', 'explosive', 'dangerous', 'threat'],
            'high': ['unauthorized', 'suspicious', 'intruder', 'violation', 'breach'],
            'medium': ['person', 'people', 'individual', 'unauthorized vehicle', 'car'],
            'safety': ['hazard', 'danger', 'unsafe', 'risk', 'emergency', 'fire', 'smoke', 'spill']
        }
        
        threat_level = "NORMAL"
        found_indicators = {}
        
        for level, keywords in security_indicators.items():
            found = [kw for kw in keywords if kw in all_text.lower()]
            if found:
                found_indicators[level] = found
                if level == 'critical':
                    threat_level = "CRITICAL"
                elif level == 'high' and threat_level not in ['CRITICAL']:
                    threat_level = "HIGH" 
                elif level in ['medium', 'safety'] and threat_level == "NORMAL":
                    threat_level = "ELEVATED"
        
        # Print threat assessment
        print(f"🚨 THREAT LEVEL: {threat_level}")
        
        if found_indicators:
            for level, indicators in found_indicators.items():
                print(f"   {level.upper()}: {', '.join(indicators)}")
        
        # Recommendations based on threat level
        if threat_level == "CRITICAL":
            print(f"🚨 IMMEDIATE ACTION REQUIRED: Deploy security response team")
        elif threat_level == "HIGH":
            print(f"⚠️  ALERT: Dispatch security personnel for investigation")
        elif threat_level == "ELEVATED":
            print(f"📞 MONITOR: Continue surveillance and prepare response")
        else:
            print(f"✅ STATUS: Area secure, continue routine monitoring")
        
        # Performance metrics
        print("-"*80)
        print(f"🧠 Model: {model_info['model_name']} ({model_info['parameters']} parameters)")
        print(f"🎯 Capabilities: {', '.join(model_info['capabilities'][:3])}")
        print("="*80)
        print()

def main(args=None):
    """
    Main function to run the SmolVLM surveillance node
    """
    rclpy.init(args=args)
    
    try:
        surveillance_node = SmolVLMSurveillanceNode()
        
        surveillance_node.get_logger().info('🧠 SmolVLM Surveillance Node started - Advanced multimodal monitoring')
        surveillance_node.get_logger().info('📸 Analyzing images with enhanced VLM capabilities')
        
        rclpy.spin(surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 SmolVLM surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running SmolVLM surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
