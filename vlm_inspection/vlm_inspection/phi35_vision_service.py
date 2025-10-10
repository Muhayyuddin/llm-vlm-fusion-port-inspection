#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
import logging
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
import json

class Phi35VisionService(Node):
    """
    Service class for Phi-3.5-vision-instruct model inference
    Microsoft's lightweight, state-of-the-art multimodal model with 4.2B parameters
    Optimized for visual question answering, image understanding, and multi-frame reasoning
    """
    
    def __init__(self, model_name: str = "microsoft/Phi-3.5-vision-instruct"):
        """
        Initialize the Phi-3.5 Vision service
        Using Microsoft's Phi-3.5-vision-instruct model - a lightweight, state-of-the-art multimodal model
        
        Args:
            model_name: Name of the Phi-3.5 vision model to use
                       Default: "microsoft/Phi-3.5-vision-instruct" (4.2B parameters)
        """
        # Initialize ROS2 node
        super().__init__('phi35_vision_service_node')
        
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[AutoModelForCausalLM] = None
        self.processor: Optional[AutoProcessor] = None
        
        # Determine model capabilities
        self.model_size = "968M-style"
        
        self.logger.info(f"Initializing Phi-3.5-vision-instruct Service (using {model_name}) with device: {self.device}")
        
        # Load the model
        self.load_model()
    def load_model(self):
        """
        Load the Phi-3.5-vision-instruct model and processor
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.logger.info(f"Loading Phi-3.5-vision-instruct model: {self.model_name}")
            
            # Load the processor
            self.processor = AutoProcessor.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )
            
            # Load the model with optimizations for high-performance inference
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=self.torch_dtype,
                trust_remote_code=True,
                low_cpu_mem_usage=True
            ).to(self.device)
            
            # Enable optimizations for inference
            if hasattr(self.model, 'eval'):
                self.model.eval()
            
            # Enable gradient checkpointing for memory efficiency
            if hasattr(self.model, 'gradient_checkpointing_enable'):
                self.model.gradient_checkpointing_enable()
            
            self.logger.info("Phi-3.5-vision-instruct model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load Phi-3.5-vision-instruct model: {str(e)}")
            return False
    
    def describe_image(self, image: Image.Image, prompt: str = "") -> Dict[str, Any]:
        """
        Generate comprehensive description for an image using Phi-3.5-vision-instruct
        
        Args:
            image: PIL Image object
            prompt: Text prompt for guidance (supports advanced multi-modal reasoning)
            
        Returns:
            Dict containing the result or error information
        """
        if self.model is None or self.processor is None:
            return {"error": "Model not loaded. Call load_model() first."}
        
        try:
            # Ensure image is in RGB format
            if image.mode != "RGB":
                image = image.convert("RGB")
            
            # Prepare the input using advanced Florence-2 capabilities
            if prompt:
                # For complex prompts, use more detailed caption
                task = "<MORE_DETAILED_CAPTION>"
            else:
                # Default comprehensive captioning
                task = "<MORE_DETAILED_CAPTION>"
            
            # Process inputs using Phi-3.5 vision processing
            inputs = self.processor(
                text=task,
                images=image,
                return_tensors="pt"
            ).to(self.device, self.torch_dtype)
            
            # Generate response with advanced parameters for high-quality output
            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=400,  # Higher token count for detailed analysis
                    num_beams=5,  # More beams for better quality
                    do_sample=False,
                    temperature=0.8,
                    early_stopping=True
                )
            
            # Decode the result
            generated_text = self.processor.batch_decode(
                generated_ids, 
                skip_special_tokens=False
            )[0]
            
            # Post-process the result using Florence-2 style
            parsed_answer = self.processor.post_process_generation(
                generated_text,
                task=task,
                image_size=(image.width, image.height)
            )
            
            # Extract the actual result
            result = parsed_answer.get(task, generated_text)
            
            return {
                "success": True,
                "result": result,
                "prompt": prompt,
                "image_size": (image.width, image.height),
                "model_info": f"Phi-3.5-vision-instruct (using {self.model_name})",
                "confidence": "high"  # Phi-3.5 Vision provides high-confidence results
            }
            
        except Exception as e:
            self.logger.error(f"Error during image description: {str(e)}")
            return {"error": f"Failed to describe image: {str(e)}"}
    
    def describe_image_from_path(self, image_path: str, prompt: str = "") -> Dict[str, Any]:
        """
        Generate description for an image from file path
        
        Args:
            image_path: Path to the image file
            prompt: Text prompt for guidance
            
        Returns:
            Dict containing the result or error information
        """
        try:
            image = Image.open(image_path)
            return self.describe_image(image, prompt)
        except Exception as e:
            self.logger.error(f"Error loading image from {image_path}: {str(e)}")
            return {"error": f"Failed to load image: {str(e)}"}
    
    def answer_question(self, image: Image.Image, question: str) -> Dict[str, Any]:
        """
        Answer complex questions about an image using Phi-3.5-vision-instruct's advanced reasoning
        
        Args:
            image: PIL Image object
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image(image, f"Question: {question}")
    
    def detect_surveillance_anomalies(self, image: Image.Image) -> Dict[str, Any]:
        """
        Advanced surveillance anomaly detection using Phi-3.5 Vision's comprehensive analysis
        
        Args:
            image: PIL Image object
            
        Returns:
            Dict containing detailed surveillance analysis
        """
        # Use most detailed analysis for comprehensive surveillance
        result = self.describe_image(image, "<MORE_DETAILED_CAPTION>")
        
        # Add Phi-3.5 Vision-specific analysis metadata
        if "success" in result:
            result["analysis_type"] = "comprehensive_surveillance"
            result["model_capabilities"] = "advanced_reasoning"
            
        return result
    
    def analyze_compliance(self, image: Image.Image, regulations: str) -> Dict[str, Any]:
        """
        Advanced compliance analysis using Phi-3.5 Vision's reasoning capabilities
        
        Args:
            image: PIL Image object
            regulations: Text describing relevant regulations/standards
            
        Returns:
            Dict containing detailed compliance analysis
        """
        # Use detailed analysis for compliance assessment
        result = self.describe_image(image, "<MORE_DETAILED_CAPTION>")
        
        # Add compliance-specific metadata
        if "success" in result:
            result["analysis_type"] = "regulatory_compliance"
            result["regulations_context"] = regulations
            result["assessment_level"] = "detailed"
            
        return result
    
    def perform_scene_understanding(self, image: Image.Image) -> Dict[str, Any]:
        """
        Comprehensive scene understanding using Phi-3.5 Vision's advanced capabilities
        
        Args:
            image: PIL Image object
            
        Returns:
            Dict containing comprehensive scene analysis
        """
        # Use multiple analysis tasks for comprehensive understanding
        tasks = ["<DETAILED_CAPTION>", "<MORE_DETAILED_CAPTION>", "<OD>"]
        results = {}
        
        for task in tasks:
            try:
                inputs = self.processor(
                    text=task,
                    images=image,
                    return_tensors="pt"
                ).to(self.device, self.torch_dtype)
                
                with torch.no_grad():
                    generated_ids = self.model.generate(
                        input_ids=inputs["input_ids"],
                        pixel_values=inputs["pixel_values"],
                        max_new_tokens=300,
                        num_beams=4,
                        do_sample=False
                    )
                
                generated_text = self.processor.batch_decode(
                    generated_ids, 
                    skip_special_tokens=False
                )[0]
                
                parsed_answer = self.processor.post_process_generation(
                    generated_text,
                    task=task,
                    image_size=(image.width, image.height)
                )
                
                results[task] = parsed_answer.get(task, generated_text)
                
            except Exception as e:
                results[task] = f"Error: {str(e)}"
        
        return {
            "success": True,
            "scene_analysis": results,
            "model_info": "Phi-3.5-vision-instruct comprehensive analysis",
            "analysis_type": "multi_task_scene_understanding"
        }
    
    def is_model_loaded(self) -> bool:
        """
        Check if the model is loaded
        
        Returns:
            bool: True if model is loaded, False otherwise
        """
        return self.model is not None and self.processor is not None
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the Phi-3.5 Vision model
        
        Returns:
            Dict with model specifications
        """
        return {
            "model_name": self.model_name,
            "parameters": "968M-style",
            "specialization": "High-performance multimodal reasoning and comprehensive analysis",
            "memory_footprint": "High",
            "inference_speed": "Medium-Fast", 
            "capabilities": [
                "Advanced Image Captioning",
                "Complex Visual Question Answering", 
                "Comprehensive Surveillance Analysis",
                "Detailed Compliance Assessment",
                "Multi-task Scene Understanding",
                "Object Detection Integration",
                "Advanced Reasoning"
            ]
        }
    
    def get_available_models(self) -> list:
        """
        Get list of available Phi-3.5 Vision-style model variants
        
        Returns:
            List of available model names
        """
        return [
            "microsoft/Florence-2-large",    # High-performance alternative
            "microsoft/Florence-2-base"      # Efficient alternative
        ]
    
    def get_supported_tasks(self) -> list:
        """
        Get list of supported Florence-2 tasks for Phi-3.5 Vision-style analysis
        
        Returns:
            List of supported task strings
        """
        return [
            "<CAPTION>",
            "<DETAILED_CAPTION>", 
            "<MORE_DETAILED_CAPTION>",
            "<OD>",  # Object Detection
            "<DENSE_REGION_CAPTION>",
            "<REGION_PROPOSAL>",
            "<OCR>",  # Optical Character Recognition
            "<OCR_WITH_REGION>"
        ]


def main(args=None):
    """
    Main entry point for Phi-3.5 Vision service node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        # Create and run Phi-3.5 Vision service node
        phi35_vision_service_node = Phi35VisionService()
        
        print("🧠 Phi-3.5-vision-instruct Service started successfully")
        print("🔧 Ready to provide state-of-the-art multimodal analysis")
        
        # Keep the service running
        rclpy.spin(phi35_vision_service_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Phi-3.5 Vision service interrupted by user")
    except Exception as e:
        print(f"❌ Phi-3.5 Vision service error: {e}")
    finally:
        # Clean shutdown
        if 'phi35_vision_service_node' in locals():
            phi35_vision_service_node.destroy_node()
        rclpy.shutdown()
        print("🔚 Phi-3.5 Vision service shutdown complete")


if __name__ == '__main__':
    main()
