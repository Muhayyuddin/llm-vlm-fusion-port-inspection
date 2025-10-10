#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
import logging
from typing import Optional, Dict, Any

class FlorenceService:
    """
    Service class for Florence-2 model inference
    """
    
    def __init__(self, model_name: str = "microsoft/Florence-2-large"):
        """
        Initialize the Florence-2 service
        
        Args:
            model_name: Name of the Florence-2 model to use
        """
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[AutoModelForCausalLM] = None
        self.processor: Optional[AutoProcessor] = None
        
        self.logger.info(f"Initializing Florence Service with device: {self.device}")
        
    def load_model(self) -> bool:
        """
        Load the Florence-2 model and processor
        
        Returns:
            bool: True if model loaded successfully, False otherwise
        """
        try:
            self.logger.info(f"Loading Florence-2 model: {self.model_name}")
            
            # Load the model
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=self.torch_dtype,
                trust_remote_code=True
            ).to(self.device)
            
            # Load the processor
            self.processor = AutoProcessor.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )
            
            self.logger.info("Florence-2 model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load Florence-2 model: {str(e)}")
            return False
    
    def describe_image(self, image: Image.Image, task: str = "<MORE_DETAILED_CAPTION>") -> Dict[str, Any]:
        """
        Generate description for an image using Florence-2
        
        Args:
            image: PIL Image object
            task: Task type for Florence-2 (e.g., "<MORE_DETAILED_CAPTION>", "<CAPTION>")
            
        Returns:
            Dict containing the result or error information
        """
        if self.model is None or self.processor is None:
            return {"error": "Model not loaded. Call load_model() first."}
        
        try:
            # Ensure image is in RGB format
            if image.mode != "RGB":
                image = image.convert("RGB")
            
            # Process inputs
            inputs = self.processor(
                text=task, 
                images=image, 
                return_tensors="pt"
            ).to(self.device, self.torch_dtype)
            
            # Generate description
            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=1024,
                    num_beams=3,
                    do_sample=False
                )
            
            # Decode the result
            generated_text = self.processor.batch_decode(
                generated_ids, 
                skip_special_tokens=False
            )[0]
            
            # Post-process the result
            result = self.processor.post_process_generation(
                generated_text, 
                task=task, 
                image_size=(image.width, image.height)
            )
            
            return {
                "success": True,
                "result": result.get(task, result),
                "task": task,
                "image_size": (image.width, image.height)
            }
            
        except Exception as e:
            self.logger.error(f"Error during image description: {str(e)}")
            return {"error": f"Failed to describe image: {str(e)}"}
    
    def describe_image_from_path(self, image_path: str, task: str = "<MORE_DETAILED_CAPTION>") -> Dict[str, Any]:
        """
        Generate description for an image from file path
        
        Args:
            image_path: Path to the image file
            task: Task type for Florence-2
            
        Returns:
            Dict containing the result or error information
        """
        try:
            image = Image.open(image_path)
            return self.describe_image(image, task)
        except Exception as e:
            self.logger.error(f"Error loading image from {image_path}: {str(e)}")
            return {"error": f"Failed to load image: {str(e)}"}
    
    def is_model_loaded(self) -> bool:
        """
        Check if the model is loaded
        
        Returns:
            bool: True if model is loaded, False otherwise
        """
        return self.model is not None and self.processor is not None
    
    def get_available_tasks(self) -> list:
        """
        Get list of available Florence-2 tasks
        
        Returns:
            List of available task strings
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
