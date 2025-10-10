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
import json

from .t5_service import T5Service
from .florence_service import FlorenceService  # For image descriptions

class T5Node(Node):
    """
    ROS2 Node for T5 text processing
    Can work with image descriptions from other VLM models
    """
    
    def __init__(self):
        super().__init__('t5_node')
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('model_name', 't5-small')
        self.declare_parameter('use_vision_model', True)  # Use Florence-2 for image descriptions
        self.declare_parameter('vision_model_name', 'microsoft/Florence-2-base')  # Lighter model for combo
        
        # Get parameters
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        use_vision_model = self.get_parameter('use_vision_model').get_parameter_value().bool_value
        vision_model_name = self.get_parameter('vision_model_name').get_parameter_value().string_value
        
        self.get_logger().info(f'Initializing T5 Node')
        self.get_logger().info(f'T5 Model: {model_name}')
        self.get_logger().info(f'Use vision model: {use_vision_model}')
        
        # Initialize T5 service
        self.t5_service = T5Service(model_name)
        
        # Initialize vision model if requested
        self.vision_service = None
        if use_vision_model:
            self.get_logger().info(f'Loading vision model: {vision_model_name}')
            self.vision_service = FlorenceService(vision_model_name)
        
        # Load models
        self.get_logger().info('Loading T5 model...')
        if not self.t5_service.load_model():
            self.get_logger().error('Failed to load T5 model')
            raise RuntimeError('Failed to load T5 model')
        
        if self.vision_service:
            self.get_logger().info('Loading vision model...')
            if not self.vision_service.load_model():
                self.get_logger().error('Failed to load vision model')
                raise RuntimeError('Failed to load vision model')
        
        self.get_logger().info('Models loaded successfully')
        
        # Create subscribers
        self.image_subscription = self.create_subscription(
            Image,
            'input_image',
            self.image_callback,
            10
        )
        
        self.text_subscription = self.create_subscription(
            String,
            'input_text',
            self.text_callback,
            10
        )
        
        # Create publishers
        self.result_publisher = self.create_publisher(String, 'text_result', 10)
        self.analysis_publisher = self.create_publisher(String, 'text_analysis', 10)
        
        self.get_logger().info('T5 Node initialized and ready')
        self.get_logger().info('Subscribed to: input_image, input_text')
        self.get_logger().info('Publishing to: text_result, text_analysis')
    
    def image_callback(self, msg: Image):
        """
        Process incoming images and generate text analysis
        
        Args:
            msg: ROS2 Image message
        """
        if not self.vision_service:
            self.get_logger().warn('Vision model not available, cannot process image')
            return
        
        try:
            # Convert ROS image to PIL format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            self.get_logger().info('Processing image with T5 pipeline...')
            
            # Get image description from vision model
            vision_result = self.vision_service.describe_image(pil_image, "<DETAILED_CAPTION>")
            
            if 'error' in vision_result:
                self.get_logger().error(f'Vision model failed: {vision_result["error"]}')
                return
            
            image_description = vision_result['result']
            self.get_logger().info(f'Image description: {image_description[:100]}...')
            
            # Process with T5 for various text tasks
            results = self.process_image_with_t5(image_description)
            
            # Publish results
            self.publish_results(results)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def text_callback(self, msg: String):
        """
        Process incoming text with T5 model
        
        Args:
            msg: ROS2 String message
        """
        try:
            input_text = msg.data
            self.get_logger().info(f'Processing text: {input_text[:50]}...')
            
            # Process with T5 for various tasks
            results = self.process_text_with_t5(input_text)
            
            # Publish results
            self.publish_results(results)
            
        except Exception as e:
            self.get_logger().error(f'Error processing text: {str(e)}')
    
    def process_image_with_t5(self, image_description):
        """
        Process image description with T5 for various tasks
        
        Args:
            image_description: Description from vision model
            
        Returns:
            dict: Processing results
        """
        results = {
            "source": "image",
            "original_description": image_description,
            "timestamp": self.get_clock().now().to_msg()
        }
        
        # 1. Summarize the image description
        summary_result = self.t5_service.summarize_text(image_description)
        if "error" not in summary_result:
            results["summary"] = summary_result["summary"]
        
        # 2. Answer specific questions about the image
        questions = [
            "What is the main subject in this image?",
            "What colors are prominent in this image?",
            "What is the setting or location?",
            "Are there any people or animals?",
            "Is there any text or writing visible?"
        ]
        
        results["qa_results"] = {}
        for question in questions:
            qa_result = self.t5_service.answer_question(image_description, question)
            if "error" not in qa_result:
                results["qa_results"][question] = qa_result["answer"]
        
        # 3. Classify the image content
        categories = ["indoor", "outdoor", "nature", "urban", "people", "animals", "objects", "landscape"]
        classification_result = self.t5_service.classify_text(image_description, categories)
        if "error" not in classification_result:
            results["classification"] = classification_result["classification"]
        
        # 4. Sentiment analysis of the scene
        sentiment_result = self.t5_service.analyze_sentiment(image_description)
        if "error" not in sentiment_result:
            results["sentiment"] = sentiment_result["sentiment"]
        
        return results
    
    def process_text_with_t5(self, text):
        """
        Process text with T5 for various tasks
        
        Args:
            text: Input text
            
        Returns:
            dict: Processing results
        """
        results = {
            "source": "text",
            "original_text": text,
            "timestamp": self.get_clock().now().to_msg()
        }
        
        # 1. Summarize the text
        if len(text) > 100:  # Only summarize longer texts
            summary_result = self.t5_service.summarize_text(text)
            if "error" not in summary_result:
                results["summary"] = summary_result["summary"]
        
        # 2. Sentiment analysis
        sentiment_result = self.t5_service.analyze_sentiment(text)
        if "error" not in sentiment_result:
            results["sentiment"] = sentiment_result["sentiment"]
        
        # 3. Classification
        categories = ["news", "opinion", "technical", "casual", "formal", "question", "instruction"]
        classification_result = self.t5_service.classify_text(text, categories)
        if "error" not in classification_result:
            results["classification"] = classification_result["classification"]
        
        # 4. Generate related questions
        question_prompt = f"generate questions about: {text}"
        question_result = self.t5_service.generate_text(question_prompt, max_length=150)
        if "error" not in question_result:
            results["generated_questions"] = question_result["generated_text"]
        
        return results
    
    def publish_results(self, results):
        """
        Publish processing results
        
        Args:
            results: Results dictionary
        """
        try:
            # Convert results to JSON string
            result_json = json.dumps(results, indent=2, default=str)
            
            # Publish detailed results
            result_msg = String()
            result_msg.data = result_json
            self.result_publisher.publish(result_msg)
            
            # Create simplified analysis message
            analysis = {
                "source": results.get("source", "unknown"),
                "summary": results.get("summary", "No summary available"),
                "sentiment": results.get("sentiment", "neutral"),
                "classification": results.get("classification", "unknown")
            }
            
            analysis_msg = String()
            analysis_msg.data = json.dumps(analysis, default=str)
            self.analysis_publisher.publish(analysis_msg)
            
            # Log key results
            self.get_logger().info("🤖 T5 Analysis Complete:")
            self.get_logger().info(f"  📝 Summary: {analysis['summary']}")
            self.get_logger().info(f"  💭 Sentiment: {analysis['sentiment']}")
            self.get_logger().info(f"  📂 Category: {analysis['classification']}")
            
        except Exception as e:
            self.get_logger().error(f'Error publishing results: {str(e)}')

def main(args=None):
    """
    Main function to run the T5 node
    """
    rclpy.init(args=args)
    
    try:
        t5_node = T5Node()
        
        t5_node.get_logger().info('🤖 T5 Node started - Ready for text processing')
        t5_node.get_logger().info('📨 Send images to: input_image')
        t5_node.get_logger().info('📝 Send text to: input_text')
        
        rclpy.spin(t5_node)
        
    except KeyboardInterrupt:
        print("\n🛑 T5 Node stopped by user")
    except Exception as e:
        print(f"❌ Error running T5 node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
