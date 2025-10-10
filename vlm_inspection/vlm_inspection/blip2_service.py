#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import Blip2Processor, Blip2ForConditionalGeneration
import logging
from typing import Optional, Dict, Any

class BLIP2Service:
    """
    Service class for BLIP2 model inference
    """
    
    def __init__(self, model_name: str = "Salesforce/blip2-opt-2.7b"):
        """
        Initialize the BLIP2 service
        
        Args:
            model_name: Name of the BLIP2 model to use
        """
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[Blip2ForConditionalGeneration] = None
        self.processor: Optional[Blip2Processor] = None
        
        self.logger.info(f"Initializing BLIP2 Service with device: {self.device}")
        
    def load_model(self) -> bool:
        """
        Load the BLIP2 model and processor
        
        Returns:
            bool: True if model loaded successfully, False otherwise
        """
        try:
            self.logger.info(f"Loading BLIP2 model: {self.model_name}")
            
            # Load the processor
            self.processor = Blip2Processor.from_pretrained(self.model_name)
            
            # Load the model
            self.model = Blip2ForConditionalGeneration.from_pretrained(
                self.model_name,
                torch_dtype=self.torch_dtype,
                device_map="auto" if self.device == "cuda" else None
            )
            
            if self.device == "cpu":
                self.model = self.model.to(self.device)
            
            self.logger.info("BLIP2 model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load BLIP2 model: {str(e)}")
            return False
    
    def describe_image(self, image: Image.Image, prompt: str = "") -> Dict[str, Any]:
        """
        Generate description for an image using BLIP2
        
        Args:
            image: PIL Image object
            prompt: Optional text prompt to guide generation
            
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
            inputs = self.processor(images=image, text=prompt, return_tensors="pt").to(self.device, self.torch_dtype)
            
            # Generate description
            with torch.no_grad():
                generated_ids = self.model.generate(
                    **inputs,
                    max_new_tokens=100,
                    num_beams=5,
                    do_sample=False,
                    temperature=1.0
                )
            
            # Decode the result
            generated_text = self.processor.batch_decode(
                generated_ids, 
                skip_special_tokens=True
            )[0].strip()
            
            return {
                "success": True,
                "result": generated_text,
                "prompt": prompt,
                "image_size": (image.width, image.height)
            }
            
        except Exception as e:
            self.logger.error(f"Error during image description: {str(e)}")
            return {"error": f"Failed to describe image: {str(e)}"}
    
    def describe_image_from_path(self, image_path: str, prompt: str = "") -> Dict[str, Any]:
        """
        Generate description for an image from file path
        
        Args:
            image_path: Path to the image file
            prompt: Optional text prompt to guide generation
            
        Returns:
            Dict containing the result or error information
        """
        try:
            image = Image.open(image_path)
            return self.describe_image(image, prompt)
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
    
    def get_available_models(self) -> list:
        """
        Get list of available BLIP2 model variants
        
        Returns:
            List of available model names
        """
        return [
            "Salesforce/blip2-opt-2.7b",      # BLIP2 with OPT-2.7B
            "Salesforce/blip2-opt-6.7b",      # BLIP2 with OPT-6.7B  
            "Salesforce/blip2-flan-t5-xl",    # BLIP2 with Flan-T5-XL
            "Salesforce/blip2-flan-t5-xxl"    # BLIP2 with Flan-T5-XXL
        ]
    
    def answer_question(self, image: Image.Image, question: str) -> Dict[str, Any]:
        """
        Answer a question about an image using BLIP2
        
        Args:
            image: PIL Image object
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image(image, f"Question: {question} Answer:")
    
    def answer_question_from_path(self, image_path: str, question: str) -> Dict[str, Any]:
        """
        Answer a question about an image from file path
        
        Args:
            image_path: Path to the image file
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image_from_path(image_path, f"Question: {question} Answer:")
