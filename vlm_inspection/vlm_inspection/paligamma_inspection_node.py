#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
import time
from .paligamma_service import PaliGammaService

class PaliGammaInspectionNode(Node):
    """
    ROS2 node for PaliGemma-based image inspection and analysis
    
    This node subscribes to image topics and performs comprehensive inspection
    using Google's PaliGemma vision-language model
    """
    
    def __init__(self):
        super().__init__('paligamma_inspection_node')
        
        # Initialize parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('inspection_frequency', 1.0)
        self.declare_parameter('inspection_type', 'general')
        self.declare_parameter('enable_vqa', True)
        self.declare_parameter('enable_surveillance', True)
        self.declare_parameter('model_name', 'microsoft/Florence-2-base-ft')
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.inspection_frequency = self.get_parameter('inspection_frequency').get_parameter_value().double_value
        self.inspection_type = self.get_parameter('inspection_type').get_parameter_value().string_value
        self.enable_vqa = self.get_parameter('enable_vqa').get_parameter_value().bool_value
        self.enable_surveillance = self.get_parameter('enable_surveillance').get_parameter_value().bool_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        self.get_logger().info(f"Initializing PaliGemma Inspection Node")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Inspection frequency: {self.inspection_frequency} Hz")
        self.get_logger().info(f"Inspection type: {self.inspection_type}")
        self.get_logger().info(f"VQA enabled: {self.enable_vqa}")
        self.get_logger().info(f"Surveillance enabled: {self.enable_surveillance}")
        self.get_logger().info(f"Model: {self.model_name}")
        
        # Initialize PaliGemma service
        self.get_logger().info("Loading PaliGemma model... This may take some time.")
        self.paligamma_service = PaliGammaService(model_name=self.model_name)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Image storage
        self.latest_image = None
        self.last_analysis_time = 0
        
        # Create publishers
        self.inspection_pub = self.create_publisher(String, 'paligamma_inspection_result', 10)
        self.caption_pub = self.create_publisher(String, 'paligamma_caption', 10)
        self.vqa_pub = self.create_publisher(String, 'paligamma_vqa_result', 10)
        self.surveillance_pub = self.create_publisher(String, 'paligamma_surveillance_alert', 10)
        
        # Create subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        # Analysis timer
        self.analysis_timer = self.create_timer(
            1.0 / self.inspection_frequency,
            self.perform_analysis
        )
        
        # VQA questions for inspection
        self.inspection_questions = [
            "What type of facility or environment is shown in this image?",
            "Are there any visible defects, damage, or maintenance issues?",
            "What safety equipment or measures are visible?",
            "Are there any people present and what are they doing?",
            "What is the overall condition of the infrastructure or equipment?",
            "Are there any potential safety hazards or risks visible?"
        ]
        
        self.get_logger().info("PaliGemma model loaded successfully")
        model_info = self.paligamma_service.get_model_info()
        self.get_logger().info(f"Model info: {model_info}")
        self.get_logger().info("🤖 PaliGemma Inspection Node initialized and ready")
        self.get_logger().info("📸 Analyzing images with Google's vision-language model")
    
    def image_callback(self, msg):
        """
        Callback for incoming images
        
        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # Convert to PIL Image
            self.latest_image = PILImage.fromarray(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def perform_analysis(self):
        """
        Perform comprehensive analysis using PaliGemma
        """
        if self.latest_image is None:
            self.get_logger().info("🤖 Waiting for image data...")
            return
        
        current_time = time.time()
        if current_time - self.last_analysis_time < (1.0 / self.inspection_frequency):
            return
        
        self.last_analysis_time = current_time
        self.get_logger().info("🤖 Performing PaliGemma inspection analysis...")
        
        try:
            # 1. Generate detailed caption
            caption_result = self.paligamma_service.generate_caption(
                self.latest_image, 
                style="detailed"
            )
            
            if 'error' not in caption_result:
                caption_msg = String()
                caption_msg.data = caption_result['result']
                self.caption_pub.publish(caption_msg)
            
            # 2. Perform inspection analysis
            inspection_result = self.paligamma_service.analyze_inspection(
                self.latest_image,
                inspection_type=self.inspection_type
            )
            
            if 'error' not in inspection_result:
                inspection_msg = String()
                inspection_msg.data = inspection_result['result']
                self.inspection_pub.publish(inspection_msg)
            
            # 3. VQA analysis if enabled
            vqa_results = {}
            if self.enable_vqa:
                for question in self.inspection_questions:
                    vqa_result = self.paligamma_service.perform_vqa(
                        self.latest_image,
                        question
                    )
                    if 'error' not in vqa_result:
                        vqa_results[question] = vqa_result['result']
            
            # 4. Surveillance analysis if enabled
            surveillance_result = None
            if self.enable_surveillance:
                surveillance_result = self.paligamma_service.analyze_surveillance(
                    self.latest_image
                )
                
                if 'error' not in surveillance_result:
                    surveillance_msg = String()
                    surveillance_msg.data = surveillance_result['result']
                    self.surveillance_pub.publish(surveillance_msg)
            
            # Generate comprehensive report
            self.print_inspection_report(
                caption_result.get('result', 'N/A'),
                inspection_result.get('result', 'N/A'),
                vqa_results,
                surveillance_result.get('result', 'N/A') if surveillance_result else 'N/A'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error during analysis: {str(e)}')
    
    def print_inspection_report(self, caption: str, inspection: str, vqa_results: dict, surveillance: str):
        """
        Print comprehensive inspection report
        
        Args:
            caption: Image caption
            inspection: Inspection analysis
            vqa_results: VQA question-answer pairs
            surveillance: Surveillance analysis
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        model_info = self.paligamma_service.get_model_info()
        
        print("\n" + "="*80)
        print(f"🤖 PALIGAMMA INSPECTION REPORT - {timestamp_str}")
        print("="*80)
        print(f"📝 IMAGE DESCRIPTION:")
        print(f"   {caption}")
        print("-"*80)
        
        print(f"🔍 INSPECTION ANALYSIS ({self.inspection_type.upper()}):")
        print(f"   {inspection}")
        print("-"*80)
        
        if vqa_results and self.enable_vqa:
            print(f"❓ DETAILED Q&A ANALYSIS:")
            for i, (question, answer) in enumerate(vqa_results.items(), 1):
                print(f"   {i}. Q: {question}")
                print(f"      A: {answer}")
                print("-"*40)
        
        if surveillance != 'N/A' and self.enable_surveillance:
            print(f"🚨 SURVEILLANCE ANALYSIS:")
            print(f"   {surveillance}")
            print("-"*80)
        
        # Analyze for key inspection indicators
        all_text = " ".join([caption, inspection, " ".join(vqa_results.values()), surveillance])
        
        # Define inspection indicators
        inspection_indicators = {
            'critical_issues': [
                'damage', 'broken', 'cracked', 'leak', 'fire', 'explosion',
                'dangerous', 'hazard', 'emergency', 'failure', 'collapsed'
            ],
            'maintenance_needed': [
                'wear', 'rust', 'corrosion', 'deterioration', 'aging',
                'maintenance', 'repair', 'replacement', 'service'
            ],
            'safety_concerns': [
                'unsafe', 'risk', 'hazard', 'violation', 'missing safety',
                'unprotected', 'exposed', 'dangerous condition'
            ],
            'quality_issues': [
                'defect', 'flaw', 'imperfection', 'substandard',
                'poor quality', 'irregularity', 'inconsistency'
            ]
        }
        
        # Determine inspection status
        inspection_status = "NORMAL"
        found_issues = {}
        priority_score = 0
        
        for category, keywords in inspection_indicators.items():
            found = [kw for kw in keywords if kw in all_text.lower()]
            if found:
                found_issues[category] = found
                if category == 'critical_issues':
                    priority_score += 10
                elif category == 'safety_concerns':
                    priority_score += 7
                elif category == 'maintenance_needed':
                    priority_score += 4
                elif category == 'quality_issues':
                    priority_score += 3
        
        if priority_score >= 10:
            inspection_status = "CRITICAL"
        elif priority_score >= 7:
            inspection_status = "HIGH PRIORITY"
        elif priority_score >= 4:
            inspection_status = "ATTENTION REQUIRED"
        elif priority_score >= 2:
            inspection_status = "MINOR ISSUES"
        
        print(f"📊 INSPECTION ASSESSMENT:")
        print(f"   Status: {inspection_status} (Priority Score: {priority_score})")
        print(f"   Model: {model_info['model_name']} ({model_info['model_size']})")
        
        if found_issues:
            for category, issues in found_issues.items():
                icon = {"critical_issues": "🔴", "safety_concerns": "⚠️", 
                       "maintenance_needed": "🔧", "quality_issues": "📋"}.get(category, "•")
                print(f"   {icon} {category.upper().replace('_', ' ')}: {', '.join(issues[:3])}")
        
        print(f"🛠️ RECOMMENDATIONS:")
        if inspection_status == "CRITICAL":
            print(f"   🚨 IMMEDIATE ACTION: Stop operations and address critical issues")
            print(f"   🚨 SAFETY: Ensure area is safe before proceeding")
        elif inspection_status == "HIGH PRIORITY":
            print(f"   ⚠️  URGENT: Schedule immediate inspection and maintenance")
            print(f"   ⚠️  MONITOR: Increase monitoring frequency")
        elif inspection_status == "ATTENTION REQUIRED":
            print(f"   📅 SCHEDULE: Plan maintenance and repairs")
            print(f"   📋 DOCUMENT: Record findings for maintenance planning")
        elif inspection_status == "MINOR ISSUES":
            print(f"   👀 MONITOR: Continue regular monitoring")
            print(f"   📝 LOG: Document minor issues for future reference")
        else:
            print(f"   ✅ CONTINUE: Maintain current inspection schedule")
            print(f"   ✅ NORMAL: No immediate action required")
        
        print("-"*80)
        print(f"🤖 ANALYSIS DETAILS:")
        print(f"   Model: {model_info['model_name']}")
        print(f"   Resolution: {model_info['input_resolution']}")
        print(f"   Specialization: {model_info['specialization']}")
        print(f"   Capabilities: {', '.join(model_info['capabilities'][:4])}")
        print("="*80)
        print()


def main(args=None):
    """
    Main entry point for PaliGemma inspection node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        # Create and run the inspection node
        inspection_node = PaliGammaInspectionNode()
        
        # Spin the node
        rclpy.spin(inspection_node)
        
    except KeyboardInterrupt:
        print("\n🛑 PaliGemma inspection monitoring stopped by user")
    except Exception as e:
        print(f"❌ PaliGemma inspection error: {e}")
    finally:
        # Clean shutdown
        if 'inspection_node' in locals():
            inspection_node.destroy_node()
        rclpy.shutdown()
        print("🔚 PaliGemma inspection node shutdown complete")


if __name__ == '__main__':
    main()
