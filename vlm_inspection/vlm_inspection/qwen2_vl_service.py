#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForVision2Seq
import logging
from typing import Optional, Dict, Any, Union
import rclpy
from rclpy.node import Node
import json

class Qwen2VLService(Node):
    """
    Service class for Qwen2-VL model inference
    Alibaba's Qwen2-VL model with excellent vision-language understanding capabilities
    Optimized for visual question answering, image understanding, and detailed visual analysis
    """
    
    def __init__(self, model_name: str = "Qwen/Qwen2-VL-2B-Instruct"):
        """
        Initialize the Qwen2-VL service
        Using Alibaba's Qwen2-VL model for advanced vision-language understanding
        
        Args:
            model_name: Name of the Qwen2-VL model to use
                       Default: "Qwen/Qwen2-VL-2B-Instruct" (2B parameters)
                       Available: "Qwen/Qwen2-VL-7B-Instruct", "Qwen/Qwen2.5-VL-2B-Instruct"
        """
        # Initialize ROS2 node
        super().__init__('qwen2_vl_service_node')
        
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[AutoModelForVision2Seq] = None
        self.processor: Optional[AutoProcessor] = None
        
        # Determine model variant
        self.variant = self._detect_variant()
        
        self.logger.info(f"Initializing Qwen2-VL Service ({self.variant}) with device: {self.device}")
        
        # Load the model
        self.load_model()
    
    def _detect_variant(self) -> str:
        """Detect Qwen model variant from model_id"""
        if "2B" in self.model_name:
            return "2B"
        elif "3B" in self.model_name:
            return "3B"
        elif "7B" in self.model_name:
            return "7B"
        elif "32B" in self.model_name:
            return "32B"
        elif "72B" in self.model_name:
            return "72B"
        else:
            return "2B"  # Default to 2B
    
    def load_model(self) -> bool:
        """
        Load the Qwen2-VL model and processor
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.logger.info(f"Loading Qwen2-VL model: {self.model_name}")
            
            # Load processor
            self.processor = AutoProcessor.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )
            
            # Load model with memory optimization
            load_kwargs = {
                "trust_remote_code": True,
                "torch_dtype": self.torch_dtype,
                "low_cpu_mem_usage": True,
            }
            
            if self.device == "auto" or self.device == "cuda":
                load_kwargs["device_map"] = "auto"
            else:
                load_kwargs["device_map"] = self.device
            
            self.model = AutoModelForVision2Seq.from_pretrained(
                self.model_name,
                **load_kwargs
            )
            
            # Enable evaluation mode
            self.model.eval()
            
            self.logger.info("Qwen2-VL model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load Qwen2-VL model: {str(e)}")
            return False
    
    def _prepare_image(self, image: Union[Image.Image, str]) -> Image.Image:
        """
        Prepare image for processing
        
        Args:
            image: PIL Image object or path to image
            
        Returns:
            PIL Image object in RGB format
        """
        if isinstance(image, str):
            pil_image = Image.open(image)
        else:
            pil_image = image
        
        if pil_image.mode != "RGB":
            pil_image = pil_image.convert("RGB")
        
        return pil_image
    
    def describe_image(self, image: Union[Image.Image, str], prompt: str = "") -> Dict[str, Any]:
        """
        Generate comprehensive description for an image using Qwen2-VL
        
        Args:
            image: PIL Image object or path to image
            prompt: Text prompt for guidance (supports complex reasoning)
            
        Returns:
            Dict containing the result or error information
        """
        if self.model is None or self.processor is None:
            return {"error": "Model not loaded. Call load_model() first."}
        
        try:
            # Prepare image
            pil_image = self._prepare_image(image)
            
            # Create question for image description
            question = prompt if prompt else "Describe this image in detail."
            
            with torch.inference_mode():
                # Format prompt for Qwen2-VL
                messages = [
                    {
                        "role": "user",
                        "content": [
                            {"type": "image", "image": pil_image},
                            {"type": "text", "text": question}
                        ]
                    }
                ]
                
                # Apply chat template
                text = self.processor.apply_chat_template(
                    messages, 
                    tokenize=False, 
                    add_generation_prompt=True
                )
                
                # Process inputs
                inputs = self.processor(
                    text=[text],
                    images=[pil_image],
                    return_tensors="pt",
                    padding=True
                )
                
                # Move inputs to appropriate device
                if hasattr(self.model, 'hf_device_map') and self.model.hf_device_map:
                    # For models with device mapping, find the first device
                    first_device = next(iter(self.model.hf_device_map.values()))
                    if first_device not in ['cpu', 'disk']:
                        inputs = inputs.to(first_device)
                elif hasattr(self.model, 'device') and str(self.model.device) != 'cpu':
                    inputs = inputs.to(self.model.device)
                elif self.device not in ["auto", "cpu"]:
                    inputs = inputs.to(self.device)
                
                # Generate with optimized parameters
                max_tokens = 512 if self.variant == "2B" else 1024
                
                generated_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=max_tokens,
                    do_sample=False,
                    temperature=0.0,
                )
                
                # Extract only new tokens
                input_tokens = inputs['input_ids'].shape[-1]
                new_tokens = generated_ids[:, input_tokens:]
                
                # Decode
                generated_text = self.processor.batch_decode(
                    new_tokens, 
                    skip_special_tokens=True
                )[0]
                
                return {
                    "success": True,
                    "result": generated_text.strip(),
                    "prompt": question,
                    "image_size": (pil_image.width, pil_image.height),
                    "model_info": f"Qwen2-VL ({self.variant})",
                    "confidence": "high"
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
    
    def answer_question(self, image: Union[Image.Image, str], question: str) -> Dict[str, Any]:
        """
        Answer complex questions about an image using Qwen2-VL's reasoning capabilities
        
        Args:
            image: PIL Image object or path to image
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image(image, question)
    
    def detect_surveillance_anomalies(self, image: Union[Image.Image, str]) -> Dict[str, Any]:
        """
        Advanced surveillance anomaly detection using Qwen2-VL's comprehensive analysis
        
        Args:
            image: PIL Image object or path to image
            
        Returns:
            Dict containing detailed surveillance analysis
        """
        surveillance_prompt = (
            "Analyze this surveillance image for any security concerns, unauthorized personnel, "
            "unusual activities, safety hazards, or policy violations. Provide a detailed assessment "
            "of what you observe and any potential risks or concerns."
        )
        
        result = self.describe_image(image, surveillance_prompt)
        
        # Add surveillance-specific metadata
        if "success" in result:
            result["analysis_type"] = "surveillance_anomaly_detection"
            result["model_capabilities"] = "advanced_visual_reasoning"
            
        return result
    
    def analyze_compliance(self, image: Union[Image.Image, str], regulations: str) -> Dict[str, Any]:
        """
        Advanced compliance analysis using Qwen2-VL's reasoning capabilities
        
        Args:
            image: PIL Image object or path to image
            regulations: Text describing relevant regulations/standards
            
        Returns:
            Dict containing detailed compliance analysis
        """
        compliance_prompt = (
            f"Analyze this image for compliance with the following regulations and standards: "
            f"{regulations}. Identify any violations, missing safety equipment, or non-compliant "
            f"activities. Provide specific observations and recommendations."
        )
        
        result = self.describe_image(image, compliance_prompt)
        
        # Add compliance-specific metadata
        if "success" in result:
            result["analysis_type"] = "regulatory_compliance"
            result["regulations_context"] = regulations
            result["assessment_level"] = "detailed"
            
        return result
    
    def perform_scene_understanding(self, image: Union[Image.Image, str]) -> Dict[str, Any]:
        """
        Comprehensive scene understanding using Qwen2-VL's capabilities
        
        Args:
            image: PIL Image object or path to image
            
        Returns:
            Dict containing comprehensive scene analysis
        """
        scene_prompts = [
            "Describe the overall scene and setting in detail.",
            "Identify all objects, people, and activities visible in the image.",
            "Analyze the spatial relationships and layout of elements in the scene.",
            "Assess the operational status and any potential issues visible."
        ]
        
        results = {}
        
        for i, prompt in enumerate(scene_prompts):
            try:
                result = self.describe_image(image, prompt)
                if "success" in result:
                    results[f"analysis_{i+1}"] = result["result"]
                else:
                    results[f"analysis_{i+1}"] = f"Error: {result.get('error', 'Unknown error')}"
            except Exception as e:
                results[f"analysis_{i+1}"] = f"Error: {str(e)}"
        
        return {
            "success": True,
            "scene_analysis": results,
            "model_info": f"Qwen2-VL ({self.variant}) comprehensive analysis",
            "analysis_type": "multi_perspective_scene_understanding"
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
        Get information about the Qwen2-VL model
        
        Returns:
            Dict with model specifications
        """
        return {
            "model_name": self.model_name,
            "parameters": f"{self.variant} parameters",
            "specialization": "Advanced vision-language understanding and reasoning",
            "memory_footprint": "Medium" if self.variant == "2B" else "High",
            "inference_speed": "Fast" if self.variant == "2B" else "Medium",
            "capabilities": [
                "Detailed Image Captioning",
                "Complex Visual Question Answering",
                "Surveillance Analysis",
                "Compliance Assessment",
                "Scene Understanding",
                "Multi-modal Reasoning",
                "Chinese and English Support"
            ]
        }
    
    def get_available_models(self) -> list:
        """
        Get list of available Qwen2-VL model variants
        
        Returns:
            List of available model names
        """
        return [
            "Qwen/Qwen2-VL-2B-Instruct",
            "Qwen/Qwen2-VL-7B-Instruct",
            "Qwen/Qwen2.5-VL-2B-Instruct",
            "Qwen/Qwen2.5-VL-3B-Instruct",
            "Qwen/Qwen2.5-VL-7B-Instruct"
        ]
    
    def get_memory_info(self) -> Dict[str, float]:
        """Get current memory usage"""
        memory_info = {}
        
        if torch.cuda.is_available() and self.device == "cuda":
            memory_info["gpu_allocated_mb"] = torch.cuda.memory_allocated() / 1024 / 1024
            memory_info["gpu_reserved_mb"] = torch.cuda.memory_reserved() / 1024 / 1024
        
        return memory_info
    
    def cleanup(self) -> None:
        """Clean up model resources"""
        if self.model is not None:
            del self.model
            self.model = None
        if self.processor is not None:
            del self.processor
            self.processor = None
        
        if torch.cuda.is_available():
            torch.cuda.empty_cache()


def main(args=None):
    """
    Main entry point for Qwen2-VL service node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        # Create and run Qwen2-VL service node
        qwen2_vl_service_node = Qwen2VLService()
        
        print("🧠 Qwen2-VL Service started successfully")
        print("🔧 Ready to provide advanced vision-language analysis")
        
        # Keep the service running
        rclpy.spin(qwen2_vl_service_node)
        
    except KeyboardInterrupt:
        print("\n🛑 Qwen2-VL service interrupted by user")
    except Exception as e:
        print(f"❌ Qwen2-VL service error: {e}")
    finally:
        # Clean shutdown
        if 'qwen2_vl_service_node' in locals():
            qwen2_vl_service_node.destroy_node()
        rclpy.shutdown()
        print("🔚 Qwen2-VL service shutdown complete")


if __name__ == '__main__':
    main()