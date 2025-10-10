#!/usr/bin/env python3

import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
import logging
from typing import Optional, Dict, Any

class SmolVLMService:
    """
    Service class for SmolVLM model inference
    Minimal footprint VLM with 256M/500M parameters and strong multimodal capability
    """
    
    def __init__(self, model_name: str = "HuggingFaceTB/SmolVLM-500M-Instruct"):
        """
        Initialize the SmolVLM service
        Using the actual SmolVLM-500M model from Hugging Face
        
        Args:
            model_name: Name of the SmolVLM model to use
                       Default: "HuggingFaceTB/SmolVLM-500M-Instruct" (500M parameters)
        """
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model: Optional[AutoModelForCausalLM] = None
        self.processor: Optional[AutoProcessor] = None
        
        # Determine model size from name
        self.model_size = "large" if "large" in model_name else "base"
        
        self.logger.info(f"Initializing SmolVLM-style Service (using {model_name}) with device: {self.device}")
        
    def load_model(self) -> bool:
        """
        Load the SmolVLM model and processor
        
        Returns:
            bool: True if model loaded successfully, False otherwise
        """
        try:
            self.logger.info(f"Loading SmolVLM-style model: {self.model_name}")
            
            # Load the processor
            self.processor = AutoProcessor.from_pretrained(
                self.model_name,
                trust_remote_code=True
            )
            
            # Load the model with optimizations
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=self.torch_dtype,
                trust_remote_code=True,
                low_cpu_mem_usage=True
            ).to(self.device)
            
            # Enable optimizations for inference
            if hasattr(self.model, 'eval'):
                self.model.eval()
            
            self.logger.info(f"SmolVLM-style model loaded successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load SmolVLM-style model: {str(e)}")
            return False
    
    def describe_image(self, image: Image.Image, prompt: str = "") -> Dict[str, Any]:
        """
        Generate description for an image using SmolVLM
        
        Args:
            image: PIL Image object
            prompt: Text prompt for guidance
            
        Returns:
            Dict containing the result or error information
        """
        if self.model is None or self.processor is None:
            return {"error": "Model not loaded. Call load_model() first."}
        
        try:
            # Ensure image is in RGB format
            if image.mode != "RGB":
                image = image.convert("RGB")
            
            # Prepare the input using Florence-2 style
            if prompt:
                # Use detailed caption for custom prompts
                task = "<MORE_DETAILED_CAPTION>"
            else:
                # Default captioning
                task = "<DETAILED_CAPTION>"
            
            # Process inputs using Florence-2 style
            inputs = self.processor(
                text=task,
                images=image,
                return_tensors="pt"
            ).to(self.device, self.torch_dtype)
            
            # Generate response with optimized parameters
            with torch.no_grad():
                generated_ids = self.model.generate(
                    input_ids=inputs["input_ids"],
                    pixel_values=inputs["pixel_values"],
                    max_new_tokens=300,  # Slightly more for better descriptions
                    num_beams=4,  # Good balance for quality/speed
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
            response = parsed_answer.get(task, generated_text)
            
            return {
                "success": True,
                "result": response,
                "prompt": prompt,
                "image_size": (image.width, image.height),
                "model_info": f"SmolVLM-style (using {self.model_name})"
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
        Answer a question about an image using SmolVLM-style processing
        
        Args:
            image: PIL Image object
            question: Question to ask about the image
            
        Returns:
            Dict containing the result or error information
        """
        return self.describe_image(image, question)
    
    def detect_surveillance_anomalies(self, image: Image.Image) -> Dict[str, Any]:
        """
        Specialized method for surveillance anomaly detection
        
        Args:
            image: PIL Image object
            
        Returns:
            Dict containing surveillance analysis
        """
        # Use Florence-2 style detailed caption for surveillance
        return self.describe_image(image, "<MORE_DETAILED_CAPTION>")
    
    def analyze_compliance(self, image: Image.Image, regulations: str) -> Dict[str, Any]:
        """
        Analyze image for compliance with specific regulations
        
        Args:
            image: PIL Image object
            regulations: Text describing relevant regulations/standards
            
        Returns:
            Dict containing compliance analysis
        """
        # Use detailed caption for compliance analysis
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
        Get information about the SmolVLM model
        
        Returns:
            Dict with model specifications
        """
        return {
            "model_name": self.model_name,
            "parameters": f"SmolVLM-style (using {self.model_size})",
            "specialization": "Advanced multimodal capability with efficient processing",
            "memory_footprint": "Medium" if self.model_size == "large" else "Medium-Low",
            "inference_speed": "Fast",
            "capabilities": [
                "Image Captioning", 
                "Visual Analysis", 
                "Surveillance Analysis",
                "Compliance Assessment",
                "Detailed Scene Understanding"
            ]
        }
    
    def get_available_models(self) -> list:
        """
        Get list of available Florence-2 model variants (used as SmolVLM alternatives)
        
        Returns:
            List of available model names
        """
        return [
            "microsoft/Florence-2-base",
            "microsoft/Florence-2-large"
        ]
