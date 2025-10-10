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

from .t5_service import T5Service
from .florence_service import FlorenceService  # For image descriptions

class T5SurveillanceNode(Node):
    """
    ROS2 Node for surveillance using T5 model with vision integration
    Combines Florence-2 for image understanding with T5 for text analysis
    """
    
    def __init__(self):
        super().__init__('t5_surveillance_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/quadrotor_1/slot0/image_raw')
        self.declare_parameter('t5_model_name', 't5-small')
        self.declare_parameter('vision_model_name', 'microsoft/Florence-2-base')  # Lighter for combo
        self.declare_parameter('analysis_rate', 1.0)  # Hz
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        t5_model_name = self.get_parameter('t5_model_name').get_parameter_value().string_value
        vision_model_name = self.get_parameter('vision_model_name').get_parameter_value().string_value
        analysis_rate = self.get_parameter('analysis_rate').get_parameter_value().double_value
        
        self.get_logger().info(f'Initializing T5 Surveillance Node')
        self.get_logger().info(f'Input topic: {input_topic}')
        self.get_logger().info(f'T5 Model: {t5_model_name}')
        self.get_logger().info(f'Vision Model: {vision_model_name}')
        self.get_logger().info(f'Analysis rate: {analysis_rate} Hz')
        
        # Initialize T5 service for text processing
        self.t5_service = T5Service(t5_model_name)
        
        # Initialize Florence-2 for image descriptions
        self.vision_service = FlorenceService(vision_model_name)
        
        # Load models
        self.get_logger().info('Loading T5 model...')
        if not self.t5_service.load_model():
            self.get_logger().error('Failed to load T5 model')
            raise RuntimeError('Failed to load T5 model')
        
        self.get_logger().info('Loading vision model for image descriptions...')
        if not self.vision_service.load_model():
            self.get_logger().error('Failed to load vision model')
            raise RuntimeError('Failed to load vision model')
        
        self.get_logger().info('Both models loaded successfully')
        
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
        
        # Surveillance prompt for T5 processing
        self.surveillance_base_prompt = (
            "I am surveying the area. This image is captured by my camera. "
            "Is there anything suspicious in the image?"
        )
        
        self.get_logger().info('T5 Surveillance Node initialized and ready')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Surveillance prompt: {self.surveillance_base_prompt}')
    
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
        Timer callback to analyze the latest image for suspicious activity using T5
        """
        if self.latest_image is None:
            self.get_logger().warn('No image available for analysis')
            return
        
        try:
            self.get_logger().info('🔍 Analyzing surveillance image with T5 pipeline...')
            
            # Step 1: Get detailed image description using Florence-2
            vision_result = self.vision_service.describe_image(
                self.latest_image, 
                "<MORE_DETAILED_CAPTION>"
            )
            
            if 'error' in vision_result:
                self.get_logger().error(f'Vision model failed: {vision_result["error"]}')
                return
            
            image_description = vision_result['result']
            self.get_logger().info(f'Image description obtained: {len(image_description)} characters')
            
            # Step 2: Process with T5 for surveillance analysis
            surveillance_results = self.process_surveillance_with_t5(image_description)
            
            # Step 3: Generate comprehensive surveillance report
            self.print_surveillance_report(image_description, surveillance_results)
            
        except Exception as e:
            self.get_logger().error(f'Error during image analysis: {str(e)}')
    
    def process_surveillance_with_t5(self, image_description):
        """
        Process image description with T5 for surveillance analysis
        
        Args:
            image_description: Description from vision model
            
        Returns:
            dict: Surveillance analysis results
        """
        results = {}
        
        # 1. Direct surveillance question answering
        surveillance_qa = self.t5_service.answer_question(
            image_description, 
            self.surveillance_base_prompt
        )
        
        if "error" not in surveillance_qa:
            results["surveillance_answer"] = surveillance_qa["answer"]
        else:
            results["surveillance_answer"] = "Could not analyze for suspicious activity"
        
        # 2. Security threat classification
        threat_categories = ["no threat", "low threat", "medium threat", "high threat"]
        threat_analysis = self.t5_service.classify_text(image_description, threat_categories)
        
        if "error" not in threat_analysis:
            results["threat_level"] = threat_analysis["classification"]
        else:
            results["threat_level"] = "unknown"
        
        # 3. Specific security questions
        security_questions = [
            "Are there any people in this image?",
            "Are there any vehicles visible?",
            "Is there any suspicious behavior or activity?",
            "Are there any weapons or dangerous objects?",
            "Is there any damage or unusual conditions?"
        ]
        
        results["security_qa"] = {}
        for question in security_questions:
            qa_result = self.t5_service.answer_question(image_description, question)
            if "error" not in qa_result:
                results["security_qa"][question] = qa_result["answer"]
        
        # 4. Generate surveillance summary
        surveillance_summary = self.t5_service.summarize_text(
            f"Surveillance report: {image_description}. Analysis: {results['surveillance_answer']}"
        )
        
        if "error" not in surveillance_summary:
            results["summary"] = surveillance_summary["summary"]
        else:
            results["summary"] = "Summary unavailable"
        
        # 5. Sentiment analysis (for detecting concerning language)
        sentiment_result = self.t5_service.analyze_sentiment(image_description)
        
        if "error" not in sentiment_result:
            results["scene_sentiment"] = sentiment_result["sentiment"]
        else:
            results["scene_sentiment"] = "neutral"
        
        return results
    
    def print_surveillance_report(self, image_description, surveillance_results):
        """
        Print formatted T5-powered surveillance report
        
        Args:
            image_description: Original image description
            surveillance_results: T5 analysis results
        """
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        
        print("\n" + "="*80)
        print(f"🤖 T5 SURVEILLANCE REPORT - {timestamp_str}")
        print("="*80)
        print(f"📍 SURVEILLANCE QUESTION: {self.surveillance_base_prompt}")
        print("-"*80)
        print(f"📸 IMAGE DESCRIPTION (Vision Model):")
        print(f"   {image_description}")
        print("-"*80)
        print(f"🔍 T5 SURVEILLANCE ANALYSIS:")
        print(f"   {surveillance_results.get('surveillance_answer', 'Analysis unavailable')}")
        print("-"*80)
        print(f"📊 THREAT CLASSIFICATION:")
        print(f"   Level: {surveillance_results.get('threat_level', 'unknown').upper()}")
        print("-"*80)
        print(f"📝 EXECUTIVE SUMMARY:")
        print(f"   {surveillance_results.get('summary', 'Summary unavailable')}")
        print("-"*80)
        print(f"🛡️  DETAILED SECURITY ASSESSMENT:")
        
        security_qa = surveillance_results.get("security_qa", {})
        for question, answer in security_qa.items():
            print(f"   Q: {question}")
            print(f"   A: {answer}")
            print()
        
        print("-"*80)
        
        # Enhanced threat assessment using T5 results and keywords
        threat_level = surveillance_results.get('threat_level', 'unknown').lower()
        surveillance_answer = surveillance_results.get('surveillance_answer', '').lower()
        
        # Combine keyword detection with T5 classification
        suspicious_keywords = [
            'weapon', 'gun', 'knife', 'suspicious', 'person', 'people', 
            'vehicle', 'car', 'truck', 'moving', 'running', 'hiding',
            'fire', 'smoke', 'damage', 'broken', 'unusual', 'strange', 'threat'
        ]
        
        combined_text = f"{image_description} {surveillance_answer}".lower()
        found_keywords = [keyword for keyword in suspicious_keywords if keyword in combined_text]
        
        # Determine final assessment
        if threat_level in ['high threat', 'medium threat'] or found_keywords:
            priority = "HIGH" if threat_level == 'high threat' else "MEDIUM"
            print(f"⚠️  SECURITY CONCERNS DETECTED - {priority} PRIORITY:")
            print(f"   🤖 T5 Threat Level: {surveillance_results.get('threat_level', 'unknown')}")
            
            if found_keywords:
                print(f"   🔍 Keywords Found: {', '.join(found_keywords)}")
            
            # Generate T5-powered recommendations
            recommendations = self.generate_t5_recommendations(surveillance_results, found_keywords)
            print(f"   📞 T5 RECOMMENDATIONS:")
            for rec in recommendations:
                print(f"     • {rec}")
        else:
            print(f"✅ NO IMMEDIATE CONCERNS DETECTED")
            print(f"   🤖 T5 Assessment: {surveillance_results.get('threat_level', 'unknown')}")
            print(f"   📊 Scene appears normal based on T5 analysis")
        
        print("-"*80)
        print(f"💭 SCENE SENTIMENT: {surveillance_results.get('scene_sentiment', 'neutral')}")
        print("="*80)
        print()
    
    def generate_t5_recommendations(self, surveillance_results, found_keywords):
        """
        Generate T5-powered recommendations based on analysis
        
        Args:
            surveillance_results: T5 analysis results
            found_keywords: Detected suspicious keywords
            
        Returns:
            list: Recommendation strings
        """
        # Create context for T5 recommendation generation
        context = f"""
        Threat level: {surveillance_results.get('threat_level', 'unknown')}
        Analysis: {surveillance_results.get('surveillance_answer', '')}
        Keywords: {', '.join(found_keywords) if found_keywords else 'none'}
        """
        
        recommendation_prompt = f"generate security recommendations for: {context}"
        
        rec_result = self.t5_service.generate_text(
            recommendation_prompt, 
            max_length=200, 
            temperature=0.3
        )
        
        if "error" not in rec_result:
            # Parse T5 generated recommendations
            generated_recs = rec_result["generated_text"].split('.')
            recommendations = [rec.strip() for rec in generated_recs if rec.strip()]
        else:
            # Fallback recommendations
            recommendations = ["Continue monitoring the area", "Review footage if available"]
        
        # Add specific recommendations based on keywords
        if 'weapon' in found_keywords or 'gun' in found_keywords:
            recommendations.insert(0, "IMMEDIATE: Alert security about potential weapons")
        
        if 'fire' in found_keywords or 'smoke' in found_keywords:
            recommendations.insert(0, "URGENT: Investigate fire/smoke reports")
        
        if 'person' in found_keywords or 'people' in found_keywords:
            recommendations.append("Track personnel movement in area")
        
        return recommendations[:5]  # Limit to top 5 recommendations

def main(args=None):
    """
    Main function to run the T5 surveillance node
    """
    rclpy.init(args=args)
    
    try:
        t5_surveillance_node = T5SurveillanceNode()
        
        t5_surveillance_node.get_logger().info('🤖 T5 Surveillance Node started - AI-powered text analysis')
        t5_surveillance_node.get_logger().info('📸 Combining vision + T5 for comprehensive surveillance')
        
        rclpy.spin(t5_surveillance_node)
        
    except KeyboardInterrupt:
        print("\n🛑 T5 Surveillance monitoring stopped by user")
    except Exception as e:
        print(f"❌ Error running T5 surveillance node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
