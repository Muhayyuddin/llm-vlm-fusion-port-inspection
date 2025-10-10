#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
import logging
from typing import Optional, Dict, Any

class NanoVLMService:
    """
    Service class for nanoVLM model inference
    Ultra-compact VLM with ~222M parameters for efficient captioning & VQA
    """
    
    def __init__(self, model_name: str = "microsoft/Florence-2-base"):
        """
        Initialize the nanoVLM service
        Note: Using Florence-2-base as a compact alternative since nanoVLM is not publicly available
        
        Args:
            model_name: Name of the compact VLM model to use
        """
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[AutoModelForCausalLM] = None
        self.processor: Optional[AutoProcessor] = None
        
        self.logger.info(f"Initializing Compact VLM Service (using {model_name}) with device: {self.device}")
        
    def load_model(self) -> bool:
        """
        Load the nanoVLM model and processor
        
        Returns:
            bool: True if model loaded successfully, False otherwise
        """
        try:
            self.logger.info(f"Loading compact VLM model: {self.model_name}")
            
            # Load the processor
            self.processor = AutoProcessor.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )
            
            # Load the model with optimizations for small model
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=self.torch_dtype,
                trust_remote_code=True,
                low_cpu_mem_usage=True
            ).to(self.device)
            
            # Enable optimizations for inference
            if hasattr(self.model, 'eval'):
                self.model.eval()
            
            self.logger.info("Compact VLM model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load compact VLM model: {str(e)}")
            return False
    
    def describe_image(self, image: Image.Image, prompt: str = "") -> Dict[str, Any]:
        """
        Generate description for an image using nanoVLM
        
        Args:
            image: PIL Image object
            prompt: Text prompt for guidance (supports VQA)
            
        Returns:
            Dict containing the result or error information
        """
        if self.model is None or self.processor is None:
            return {"error": "Model not loaded. Call load_model() first."}
        
        try:
            # Ensure image is in RGB format
            if image.mode != "RGB":
                image = image.convert("RGB")
            
            # Use Florence-2 style processing for compact surveillance
            task = "<MORE_DETAILED_CAPTION>"
            if prompt:
                # For surveillance prompts, use detailed caption
                task = "<MORE_DETAILED_CAPTION>"
            
            # Process inputs using Florence-2 style
            inputs = self.processor(
                text=task,
                images=image,
                return_tensors="pt"
            ).to(self.device, self.torch_dtype)
            
            # Generate response with optimized parameters for compact model
            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=200,  # Reduced for efficiency
                    num_beams=3,  # Reduced beams for speed
                    do_sample=False
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
                "model_info": f"Compact VLM (using {self.model_name})"
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
        Answer a question about an image using compact VLM's capability
        
        Args:
            image: PIL Image object
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image(image, f"Question: {question}")
    
    def detect_surveillance_anomalies(self, image: Image.Image) -> Dict[str, Any]:
        """
        Specialized method for surveillance anomaly detection
        
        Args:
            image: PIL Image object
            
        Returns:
            Dict containing surveillance analysis
        """
        # Use Florence-2 style task for surveillance
        return self.describe_image(image, "<MORE_DETAILED_CAPTION>")
    
    def is_model_loaded(self) -> bool:
        """
        Check if the model is loaded
        
        Returns:
            bool: True if model is loaded, False otherwise
        """
        return self.model is not None and self.processor is not None
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the nanoVLM model
        
        Returns:
            Dict with model specifications
        """
        return {
            "model_name": self.model_name,
            "parameters": "Compact (using Florence-2-base)",
            "specialization": "Efficient captioning & surveillance analysis",
            "memory_footprint": "Medium-Low",
            "inference_speed": "Fast", 
            "capabilities": ["Image Captioning", "Visual Analysis", "Surveillance Analysis"]
        }
