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

from .phi35_vision_service import Phi35VisionService

class Phi35VisionSurveillanceNode(Node):
    """
    ROS2 Node for surveillance using Phi-3.5-vision-instruct model (4.2B parameters)
    State-of-the-art multimodal surveillance with advanced vision-language capabilities
    Subscribes to quadrotor camera feed and performs comprehensive surveillance analysis
    """
    
    def __init__(self):
        super().__init__('phi35_vision_surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('model_name', 'microsoft/Phi-3.5-vision-instruct')
        self.declare_parameter('analysis_rate', 1.0)  # Moderate rate for high-quality analysis
        self.declare_parameter('enable_compliance_check', True)
        self.declare_parameter('enable_scene_understanding', True)
        self.declare_parameter('enable_advanced_vqa', True)
        self.declare_parameter('threat_sensitivity', 'high')
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        enable_compliance = self.get_parameter('enable_compliance_check').get_parameter_value().bool_value
        enable_scene_understanding = self.get_parameter('enable_scene_understanding').get_parameter_value().bool_value
        enable_advanced_vqa = self.get_parameter('enable_advanced_vqa').get_parameter_value().bool_value
        threat_sensitivity = self.get_parameter('threat_sensitivity').get_parameter_value().string_value
        
        self.get_logger().info(f'Initializing Phi-3.5-vision-instruct Surveillance Node')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        self.get_logger().info(f'Compliance checking: {enable_compliance}')
        self.get_logger().info(f'Scene understanding: {enable_scene_understanding}')
        self.get_logger().info(f'Advanced VQA: {enable_advanced_vqa}')
        self.get_logger().info(f'Threat sensitivity: {threat_sensitivity}')
        
        # Initialize Phi-3.5 Vision service
        self.phi35_vision_service = Phi35VisionService(model_name)
        self.enable_compliance = enable_compliance
        self.enable_scene_understanding = enable_scene_understanding
        self.enable_advanced_vqa = enable_advanced_vqa
        self.threat_sensitivity = threat_sensitivity
        
        # Load the model
        self.get_logger().info('Loading Phi-3.5-vision-instruct model... This may take some time for state-of-the-art model.')
        if not self.phi35_vision_service.load_model():
            self.get_logger().error('Failed to load Phi-3.5-vision-instruct model')
            raise RuntimeError('Failed to load Phi-3.5-vision-instruct model')
        
        self.get_logger().info('Phi-3.5-vision-instruct model loaded successfully')
        
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
        
        # Advanced surveillance configuration for Phi-3.5 Vision
        self.comprehensive_surveillance_prompt = (
            "Conduct a comprehensive security analysis of this maritime port surveillance image. "
            "Perform detailed assessment covering:\n"
            "1. Personnel identification and authorization status\n"
            "2. Vehicle compliance and operational appropriateness\n" 
            "3. Safety hazards and environmental risk factors\n"
            "4. Security violations, suspicious activities, or policy breaches\n"
            "5. Operational efficiency and workflow compliance\n"
            "6. Infrastructure integrity and maintenance status\n"
            "Provide detailed observations with confidence assessments and specific location references."
        )
        
        self.advanced_vqa_questions = [
            "How many personnel are visible and what specific activities are they engaged in?",
            "What types of vehicles and equipment are present, and are they operating within authorized parameters?",
            "Are there any immediate safety hazards, environmental risks, or emergency situations visible?",
            "Do you observe any security violations, unauthorized access, or suspicious behavioral patterns?",
            "What is the current operational status and efficiency level of this port facility area?",
            "Are there any infrastructure defects, equipment malfunctions, or maintenance requirements visible?",
            "What is the overall compliance status with maritime safety and security protocols?",
            "Can you identify any potential process improvements or optimization opportunities?"
        ]
        
        self.maritime_safety_regulations = (
            "Comprehensive Maritime Port Safety and Security Regulations:\n"
            "- All personnel must wear high-visibility safety equipment and proper PPE\n"
            "- Vehicle access limited to authorized operational trucks, forklifts, and service vehicles\n" 
            "- No unauthorized personnel in restricted operational zones\n"
            "- Smoking and open flames prohibited near cargo and fuel areas\n"
            "- Container stacking must follow international safety height and stability guidelines\n"
            "- Clear emergency evacuation routes must be maintained at all times\n"
            "- All equipment must display current safety inspection certifications\n"
            "- Security cameras must have unobstructed views of critical areas\n"
            "- Environmental spill containment systems must be readily accessible\n"
            "- Communication devices required for all operational personnel"
        )
        
        self.get_logger().info('Phi-3.5 Vision Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Model info: {self.phi35_vision_service.get_model_info()}')
    
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
        Timer callback to perform comprehensive surveillance analysis
        """
        if self.latest_image is None:
            self.get_logger().warn('No image available for analysis')
            return
        
        try:
            self.get_logger().info('🧠 Performing comprehensive surveillance analysis with Phi-3.5 Vision...')
            
            # 1. Main comprehensive surveillance analysis
            surveillance_result = self.phi35_vision_service.detect_surveillance_anomalies(self.latest_image)
            
            if 'error' in surveillance_result:
                self.get_logger().error(f'Phi-3.5 Vision surveillance analysis failed: {surveillance_result["error"]}')
                return
            
            surveillance_analysis = surveillance_result['result']
            
            # 2. Scene understanding analysis if enabled
            scene_analysis = {}
            if self.enable_scene_understanding:
                scene_result = self.phi35_vision_service.perform_scene_understanding(self.latest_image)
                if 'error' not in scene_result:
                    scene_analysis = scene_result.get('scene_analysis', {})
            
            # 3. Advanced VQA analysis if enabled
            vqa_results = {}
            if self.enable_advanced_vqa:
                for question in self.advanced_vqa_questions:
                    vqa_result = self.phi35_vision_service.answer_question(self.latest_image, question)
                    if 'error' not in vqa_result:
                        vqa_results[question] = vqa_result['result']
            
            # 4. Compliance analysis if enabled
            compliance_analysis = ""
            if self.enable_compliance:
                compliance_result = self.phi35_vision_service.analyze_compliance(
                    self.latest_image, 
                    self.maritime_safety_regulations
                )
                if 'error' not in compliance_result:
                    compliance_analysis = str(compliance_result.get('result', ''))
            
            # Generate comprehensive surveillance report
            self.print_comprehensive_surveillance_report(
                surveillance_analysis, 
                scene_analysis,
                vqa_results, 
                compliance_analysis
            )
            
        except Exception as e:
            self.get_logger().error(f'Error during comprehensive analysis: {str(e)}')
    
    def print_comprehensive_surveillance_report(self, surveillance_analysis: str, scene_analysis: dict, 
                                               vqa_results: dict, compliance_analysis: str):
        """
        Print comprehensive surveillance report with advanced analysis
        
        Args:
            surveillance_analysis: Main surveillance analysis
            scene_analysis: Multi-task scene understanding results
            vqa_results: Advanced VQA question-answer pairs
            compliance_analysis: Regulatory compliance assessment
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        model_info = self.phi35_vision_service.get_model_info()
        
        print("\n" + "="*90)
        print(f"🧠 PHI-3.5 VISION COMPREHENSIVE SURVEILLANCE REPORT - {timestamp_str}")
        print("="*90)
        print(f"🔍 PRIMARY SURVEILLANCE ANALYSIS:")
        print(f"   {surveillance_analysis}")
        print("-"*90)
        
        # Display scene understanding results
        if scene_analysis:
            print(f"🎯 MULTI-TASK SCENE UNDERSTANDING:")
            for task, result in scene_analysis.items():
                if isinstance(result, str) and len(result) > 0:
                    task_name = task.replace('<', '').replace('>', '').replace('_', ' ').title()
                    print(f"   • {task_name}: {result[:200]}{'...' if len(result) > 200 else ''}")
            print("-"*90)
        
        # Display advanced VQA results
        if vqa_results:
            print(f"❓ ADVANCED VISUAL Q&A ANALYSIS:")
            for i, (question, answer) in enumerate(vqa_results.items(), 1):
                print(f"   {i}. Q: {question}")
                print(f"      A: {answer}")
                print("-"*45)
        
        # Display compliance analysis
        if compliance_analysis:
            print(f"⚖️  REGULATORY COMPLIANCE ASSESSMENT:")
            print(f"   {compliance_analysis}")
            print("-"*90)
        
        # Advanced threat assessment with Phi-3.5 Vision capabilities
        all_text = " ".join([
            str(surveillance_analysis),
            " ".join([str(v) for v in scene_analysis.values() if v]) if scene_analysis else "",
            " ".join([str(v) for v in vqa_results.values() if v]) if vqa_results else "",
            str(compliance_analysis)
        ])
        
        # Comprehensive security indicators
        security_indicators = {
            'critical': [
                'weapon', 'gun', 'knife', 'explosive', 'bomb', 'dangerous device',
                'fire', 'explosion', 'emergency', 'medical emergency', 'accident'
            ],
            'high': [
                'unauthorized', 'suspicious', 'intruder', 'violation', 'breach', 'trespassing',
                'theft', 'vandalism', 'sabotage', 'unauthorized access'
            ],
            'medium': [
                'person', 'people', 'individual', 'unauthorized vehicle', 'improper equipment',
                'safety concern', 'maintenance issue', 'protocol deviation'
            ],
            'safety': [
                'hazard', 'danger', 'unsafe', 'risk', 'spill', 'leak', 'structural damage',
                'equipment failure', 'environmental concern'
            ],
            'compliance': [
                'violation', 'non-compliant', 'improper', 'missing equipment', 'inadequate',
                'regulation breach', 'safety equipment missing'
            ]
        }
        
        # Determine comprehensive threat level
        threat_level = "NORMAL"
        threat_score = 0
        found_indicators = {}
        
        for level, keywords in security_indicators.items():
            found = [kw for kw in keywords if kw in all_text.lower()]
            if found:
                found_indicators[level] = found
                if level == 'critical':
                    threat_score += 10
                elif level == 'high':
                    threat_score += 7
                elif level == 'medium':
                    threat_score += 4
                elif level in ['safety', 'compliance']:
                    threat_score += 3
        
        # Determine final threat level based on score and sensitivity
        sensitivity_multiplier = {
            'low': 0.7,
            'medium': 1.0,
            'high': 1.3
        }.get(self.threat_sensitivity, 1.0)
        
        adjusted_score = threat_score * sensitivity_multiplier
        
        if adjusted_score >= 10:
            threat_level = "CRITICAL"
        elif adjusted_score >= 7:
            threat_level = "HIGH"
        elif adjusted_score >= 4:
            threat_level = "ELEVATED"
        elif adjusted_score >= 2:
            threat_level = "MODERATE"
        
        # Print comprehensive threat assessment
        print(f"🚨 COMPREHENSIVE THREAT ASSESSMENT:")
        print(f"   Threat Level: {threat_level} (Score: {adjusted_score:.1f})")
        print(f"   Sensitivity Setting: {self.threat_sensitivity.upper()}")
        
        if found_indicators:
            for level, indicators in found_indicators.items():
                icon = {"critical": "🔴", "high": "🟠", "medium": "🟡", "safety": "⚠️", "compliance": "📋"}.get(level, "•")
                print(f"   {icon} {level.upper()}: {', '.join(indicators[:5])}")
        
        # Comprehensive recommendations
        print(f"📋 RECOMMENDED ACTIONS:")
        if threat_level == "CRITICAL":
            print(f"   🚨 IMMEDIATE EMERGENCY RESPONSE: Deploy security and emergency teams")
            print(f"   🚨 EVACUATE: Clear area and establish security perimeter")
        elif threat_level == "HIGH":
            print(f"   ⚠️  URGENT: Dispatch security personnel for immediate investigation")
            print(f"   ⚠️  MONITOR: Increase surveillance frequency and alertness level")
        elif threat_level == "ELEVATED":
            print(f"   📞 INVESTIGATE: Send security team for detailed area inspection")
            print(f"   📞 DOCUMENT: Record incident details for security analysis")
        elif threat_level == "MODERATE":
            print(f"   👀 OBSERVE: Continue enhanced monitoring of the area")
            print(f"   👀 PREPARE: Alert relevant personnel for potential response")
        else:
            print(f"   ✅ CONTINUE: Maintain standard surveillance protocols")
            print(f"   ✅ ROUTINE: Area secure, continue normal operations")
        
        # Performance and model information
        print("-"*90)
        print(f"🧠 ANALYSIS DETAILS:")
        print(f"   Model: {model_info['model_name']} ({model_info['parameters']} parameters)")
        print(f"   Capabilities: {', '.join(model_info['capabilities'][:4])}")
        print(f"   Analysis Type: Comprehensive Multi-Modal Surveillance")
        print(f"   Confidence Level: HIGH (Phi-3.5-vision-instruct)")
        print("="*90)
        print()

def main(args=None):
    """
    Main function to run the Phi-3.5 Vision surveillance node
    """
    rclpy.init(args=args)
    
    try:
        surveillance_node = Phi35VisionSurveillanceNode()
        
        surveillance_node.get_logger().info('🧠 Phi-3.5-vision-instruct Surveillance Node started - Comprehensive monitoring')
        surveillance_node.get_logger().info('📸 Analyzing images with state-of-the-art multimodal reasoning')
        
        rclpy.spin(surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Phi-3.5 Vision surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running Phi-3.5 Vision surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
