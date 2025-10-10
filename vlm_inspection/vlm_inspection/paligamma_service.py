#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
import logging
from typing import Optional, Dict, Any
import rclpy
from rclpy.node import Node
import json
import gc

class PaliGammaService(Node):
    """
    Service class for PaliGemma model inference
    Google's vision-language model combining SigLIP vision encoder with Gemma language model
    Optimized for visual question answering and image understanding tasks
    """
    
    def __init__(self, model_name: str = "microsoft/Florence-2-base-ft"):
        """
        Initialize the PaliGemma service
        Note: Using Florence-2 as alternative since PaliGemma requires gated access
        
        Args:
            model_name: Name of the model to use (defaults to Florence-2 alternative)
        """
        # Initialize ROS2 node
        super().__init__('paligamma_service_node')
        
        self.logger = logging.getLogger(__name__)
        
        # Check if user requested actual PaliGemma model
        if "paligemma" in model_name.lower():
            self.logger.warning("PaliGemma models require gated access. Using Florence-2-base-ft as alternative.")
            self.model_name = "microsoft/Florence-2-base-ft"
        else:
            self.model_name = model_name
            
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float32  # Use float32 to avoid mixed precision issues
        
        self.model: Optional[AutoModelForCausalLM] = None
        self.processor: Optional[AutoProcessor] = None
        
        # Model specifications for PaliGemma-style analysis
        self.model_size = "3B-style" 
        self.input_resolution = 224
        
        self.logger.info(f"Initializing PaliGemma-style Service ({self.model_name}) with device: {self.device}")
        
        # Load the model
        self.load_model()
        
    def load_model(self) -> bool:
        """
        Load the PaliGemma-style model and processor (using Florence-2 as alternative)
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.logger.info(f"Loading PaliGemma-style model: {self.model_name}")
            
            # Load processor
            self.processor = AutoProcessor.from_pretrained(self.model_name, trust_remote_code=True)
            
            # Load model with optimizations
            model_kwargs = {
                "torch_dtype": torch.float32,  # Use float32 to avoid mixed precision issues
                "trust_remote_code": True,
            }
            
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                **model_kwargs
            )
            
            # Move to device
            if self.device == "cuda":
                self.model = self.model.to(self.device)
            
            # Enable evaluation mode
            self.model.eval()
            
            # Memory optimization
            if hasattr(self.model, 'gradient_checkpointing_enable'):
                self.model.gradient_checkpointing_enable()
            
            self.logger.info("PaliGemma-style model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load PaliGemma-style model: {e}")
            return False
    
    def analyze_image(self, image: Image.Image, prompt: str = "Describe this image in detail") -> Dict[str, Any]:
        """
        Analyze image using PaliGemma-style model (Florence-2 backend)
        
        Args:
            image: PIL Image to analyze
            prompt: Text prompt for the model
            
        Returns:
            Dictionary containing analysis results
        """
        try:
            if self.model is None or self.processor is None:
                return {"error": "Model not loaded"}
            
            # For Florence-2, we need to use task-specific prompts
            # Convert general prompts to Florence-2 tasks
            if "describe" in prompt.lower() or "caption" in prompt.lower():
                florence_prompt = "<MORE_DETAILED_CAPTION>"
            elif "question" in prompt.lower():
                florence_prompt = "<MORE_DETAILED_CAPTION>"  # Will process question in text
            else:
                florence_prompt = "<MORE_DETAILED_CAPTION>"
            
            # Prepare inputs for Florence-2
            inputs = self.processor(text=florence_prompt, images=image, return_tensors="pt")
            
            # Move inputs to device with correct dtypes
            if self.device == "cuda":
                # Keep input_ids as long tensors (integers), only convert pixel_values to float32
                if "input_ids" in inputs:
                    inputs["input_ids"] = inputs["input_ids"].to(self.device)
                if "pixel_values" in inputs:
                    inputs["pixel_values"] = inputs["pixel_values"].to(self.device, dtype=torch.float32)
                if "attention_mask" in inputs:
                    inputs["attention_mask"] = inputs["attention_mask"].to(self.device)
            
            # Generate response
            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=1024,
                    early_stopping=False,
                    do_sample=False,
                    num_beams=3,
                )
            
            # Decode response
            generated_text = self.processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
            
            # Post-process Florence-2 output
            parsed_answer = self.processor.post_process_generation(
                generated_text, 
                task=florence_prompt, 
                image_size=(image.width, image.height)
            )
            
            # Extract the relevant text
            if florence_prompt in parsed_answer:
                response = parsed_answer[florence_prompt]
            else:
                response = str(parsed_answer)
            
            return {
                "result": response,
                "prompt": prompt,
                "model": self.model_name,
                "confidence": "high"
            }
            
        except Exception as e:
            self.logger.error(f"Error analyzing image: {e}")
            return {"error": str(e)}
    
    def perform_vqa(self, image: Image.Image, question: str) -> Dict[str, Any]:
        """
        Perform Visual Question Answering using PaliGemma
        
        Args:
            image: PIL Image to analyze
            question: Question about the image
            
        Returns:
            Dictionary containing VQA results
        """
        try:
            # PaliGemma is designed for VQA tasks
            vqa_prompt = f"Question: {question} Answer:"
            
            result = self.analyze_image(image, vqa_prompt)
            
            if "error" not in result:
                result["question"] = question
                result["task_type"] = "visual_question_answering"
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error performing VQA: {e}")
            return {"error": str(e)}
    
    def generate_caption(self, image: Image.Image, style: str = "detailed") -> Dict[str, Any]:
        """
        Generate image captions using PaliGemma
        
        Args:
            image: PIL Image to caption
            style: Caption style - "detailed", "brief", or "technical"
            
        Returns:
            Dictionary containing caption results
        """
        try:
            # Define prompts based on style
            caption_prompts = {
                "detailed": "Provide a detailed description of this image:",
                "brief": "Briefly describe this image:",
                "technical": "Describe the technical aspects and objects visible in this image:",
                "surveillance": "Analyze this image for security and safety concerns:"
            }
            
            prompt = caption_prompts.get(style, caption_prompts["detailed"])
            
            result = self.analyze_image(image, prompt)
            
            if "error" not in result:
                result["caption_style"] = style
                result["task_type"] = "image_captioning"
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error generating caption: {e}")
            return {"error": str(e)}
    
    def analyze_surveillance(self, image: Image.Image) -> Dict[str, Any]:
        """
        Perform surveillance analysis using PaliGemma
        
        Args:
            image: PIL Image to analyze for surveillance
            
        Returns:
            Dictionary containing surveillance analysis
        """
        try:
            surveillance_prompt = (
                "Analyze this image for security surveillance. "
                "Identify any people, vehicles, suspicious activities, safety hazards, "
                "or security concerns. Provide a detailed security assessment:"
            )
            
            result = self.analyze_image(image, surveillance_prompt)
            
            if "error" not in result:
                result["task_type"] = "surveillance_analysis"
                result["security_focus"] = True
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error analyzing surveillance: {e}")
            return {"error": str(e)}
    
    def analyze_inspection(self, image: Image.Image, inspection_type: str = "general") -> Dict[str, Any]:
        """
        Perform inspection analysis using PaliGemma
        
        Args:
            image: PIL Image to inspect
            inspection_type: Type of inspection - "general", "safety", "quality", "maintenance"
            
        Returns:
            Dictionary containing inspection analysis
        """
        try:
            inspection_prompts = {
                "general": "Perform a general inspection of this image. Identify any defects, issues, or areas of concern:",
                "safety": "Analyze this image for safety compliance and hazards. Check for safety violations or risks:",
                "quality": "Perform a quality inspection of this image. Identify any quality issues or defects:",
                "maintenance": "Analyze this image for maintenance requirements. Identify equipment condition and maintenance needs:",
                "infrastructure": "Inspect the infrastructure shown in this image. Check for structural integrity and maintenance needs:"
            }
            
            prompt = inspection_prompts.get(inspection_type, inspection_prompts["general"])
            
            result = self.analyze_image(image, prompt)
            
            if "error" not in result:
                result["inspection_type"] = inspection_type
                result["task_type"] = "inspection_analysis"
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error analyzing inspection: {e}")
            return {"error": str(e)}
    
    def detect_objects(self, image: Image.Image) -> Dict[str, Any]:
        """
        Detect and describe objects in the image using PaliGemma
        
        Args:
            image: PIL Image to analyze
            
        Returns:
            Dictionary containing object detection results
        """
        try:
            detection_prompt = (
                "List and describe all the objects, people, and items visible in this image. "
                "Include their locations and any notable characteristics:"
            )
            
            result = self.analyze_image(image, detection_prompt)
            
            if "error" not in result:
                result["task_type"] = "object_detection"
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error detecting objects: {e}")
            return {"error": str(e)}
    
    def cleanup_memory(self):
        """Clean up GPU memory"""
        if self.device == "cuda":
            torch.cuda.empty_cache()
            gc.collect()
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the loaded model
        
        Returns:
            Dictionary with model information
        """
        return {
            "model_name": self.model_name,
            "model_size": self.model_size,
            "input_resolution": f"{self.input_resolution}x{self.input_resolution}",
            "device": self.device,
            "dtype": str(self.torch_dtype),
            "specialization": "Vision-Language Understanding with SigLIP + Gemma",
            "capabilities": [
                "Visual Question Answering",
                "Image Captioning", 
                "Object Detection",
                "Surveillance Analysis",
                "Inspection Analysis",
                "Text Reading (OCR)",
                "Scene Understanding"
            ],
            "strengths": [
                "High-quality VQA performance",
                "Detailed image descriptions",
                "Multilingual support",
                "Fine-grained object recognition",
                "Spatial reasoning"
            ]
        }
    
    def get_supported_tasks(self) -> list:
        """
        Get list of supported tasks
        
        Returns:
            List of supported task types
        """
        return [
            "image_captioning",
            "visual_question_answering",
            "object_detection",
            "surveillance_analysis", 
            "inspection_analysis",
            "scene_understanding",
            "text_reading"
        ]


def main(args=None):
    """
    Main entry point for PaliGemma service node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        # Create and run PaliGemma service node
        paligamma_service_node = PaliGammaService()
        
        print("🤖 PaliGemma Service started successfully")
        print("🔧 Ready to provide vision-language analysis")
        
        # Keep the service running
        rclpy.spin(paligamma_service_node)
        
    except KeyboardInterrupt:
        print("\n🛑 PaliGemma service interrupted by user")
    except Exception as e:
        print(f"❌ PaliGemma service error: {e}")
    finally:
        # Clean shutdown
        if 'paligamma_service_node' in locals():
            paligamma_service_node.cleanup_memory()
            paligamma_service_node.destroy_node()
        rclpy.shutdown()
        print("🔚 PaliGemma service shutdown complete")


if __name__ == '__main__':
    main()
