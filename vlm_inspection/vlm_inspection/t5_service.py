#!/usr/bin/env python3

import torch
from transformers import T5Tokenizer, T5ForConditionalGeneration
from PIL import Image
import logging
import gc

class T5Service:
    """
    Service class for T5 text generation model integration
    Note: T5 is primarily a text-to-text model, so for image analysis,
    we'll combine it with a vision encoder or use it for text processing
    of image descriptions from other models.
    """
    
    def __init__(self, model_name="t5-small"):
        """
        Initialize T5 service
        
        Args:
            model_name: T5 model name (t5-small, t5-base, t5-large, etc.)
        """
        self.model_name = model_name
        self.model = None
        self.tokenizer = None
        self.loaded = False
        
        # Setup logging first
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize device after logger
        self.device = self._get_device()
        
    def _get_device(self):
        """Get the appropriate device for the model"""
        try:
            if torch.cuda.is_available():
                device = torch.device('cuda')
                self.logger.info(f"Using GPU: {torch.cuda.get_device_name()}")
                return device
        except Exception as e:
            self.logger.warning(f"CUDA not available or error: {e}")
        
        device = torch.device('cpu')
        self.logger.info("Using CPU")
        return device
    
    def load_model(self):
        """
        Load T5 model and tokenizer
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.logger.info(f"Loading T5 model: {self.model_name}")
            
            # Load tokenizer
            self.tokenizer = T5Tokenizer.from_pretrained(self.model_name)
            
            # Load model with better error handling
            try:
                if self.device.type == 'cuda':
                    self.model = T5ForConditionalGeneration.from_pretrained(
                        self.model_name,
                        torch_dtype=torch.float16,
                        device_map="auto"
                    )
                else:
                    self.model = T5ForConditionalGeneration.from_pretrained(
                        self.model_name,
                        torch_dtype=torch.float32
                    )
                    self.model = self.model.to(self.device)
            except Exception as gpu_error:
                self.logger.warning(f"GPU loading failed: {gpu_error}, falling back to CPU")
                self.device = torch.device('cpu')
                self.model = T5ForConditionalGeneration.from_pretrained(
                    self.model_name,
                    torch_dtype=torch.float32
                )
                self.model = self.model.to(self.device)
            
            self.model.eval()
            self.loaded = True
            
            self.logger.info(f"T5 model loaded successfully on {self.device}")
            return True
            
        except Exception as e:
            self.logger.error(f"Error loading T5 model: {str(e)}")
            return False
    
    def generate_text(self, input_text, max_length=512, temperature=0.7, do_sample=True):
        """
        Generate text using T5 model
        
        Args:
            input_text: Input text prompt
            max_length: Maximum length of generated text
            temperature: Sampling temperature
            do_sample: Whether to use sampling
            
        Returns:
            dict: Generated text result or error
        """
        if not self.loaded:
            return {"error": "T5 model not loaded"}
        
        try:
            # Tokenize input
            inputs = self.tokenizer(
                input_text, 
                return_tensors="pt", 
                max_length=512, 
                truncation=True
            ).to(self.device)
            
            # Generate text
            with torch.no_grad():
                outputs = self.model.generate(
                    inputs.input_ids,
                    max_length=max_length,
                    temperature=temperature,
                    do_sample=do_sample,
                    pad_token_id=self.tokenizer.eos_token_id,
                    num_return_sequences=1
                )
            
            # Decode output
            generated_text = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
            
            return {
                "generated_text": generated_text,
                "input_text": input_text
            }
            
        except Exception as e:
            self.logger.error(f"Error generating text: {str(e)}")
            return {"error": f"Text generation failed: {str(e)}"}
    
    def answer_question(self, context, question):
        """
        Answer a question based on context using T5
        
        Args:
            context: Context text
            question: Question to answer
            
        Returns:
            dict: Answer result or error
        """
        # Format as T5 question-answering task
        input_text = f"question: {question} context: {context}"
        
        result = self.generate_text(input_text, max_length=256, temperature=0.3)
        
        if "error" in result:
            return result
        
        return {
            "answer": result["generated_text"],
            "question": question,
            "context": context
        }
    
    def summarize_text(self, text, max_length=150):
        """
        Summarize text using T5
        
        Args:
            text: Text to summarize
            max_length: Maximum length of summary
            
        Returns:
            dict: Summary result or error
        """
        input_text = f"summarize: {text}"
        
        result = self.generate_text(input_text, max_length=max_length, temperature=0.3)
        
        if "error" in result:
            return result
        
        return {
            "summary": result["generated_text"],
            "original_text": text
        }
    
    def analyze_sentiment(self, text):
        """
        Analyze sentiment of text using T5
        
        Args:
            text: Text to analyze
            
        Returns:
            dict: Sentiment analysis result or error
        """
        input_text = f"sentiment: {text}"
        
        result = self.generate_text(input_text, max_length=50, temperature=0.1)
        
        if "error" in result:
            return result
        
        return {
            "sentiment": result["generated_text"],
            "text": text
        }
    
    def classify_text(self, text, categories):
        """
        Classify text into categories using T5
        
        Args:
            text: Text to classify
            categories: List of possible categories
            
        Returns:
            dict: Classification result or error
        """
        categories_str = ", ".join(categories)
        input_text = f"classify into {categories_str}: {text}"
        
        result = self.generate_text(input_text, max_length=50, temperature=0.1)
        
        if "error" in result:
            return result
        
        return {
            "classification": result["generated_text"],
            "text": text,
            "categories": categories
        }
    
    def process_surveillance_text(self, image_description, question="Is there anything suspicious?"):
        """
        Process surveillance text using T5 for analysis
        
        Args:
            image_description: Description of the image from another model
            question: Surveillance question
            
        Returns:
            dict: Surveillance analysis result or error
        """
        # Create surveillance analysis prompt
        surveillance_prompt = (
            f"analyze surveillance image description for security concerns: "
            f"{image_description} question: {question}"
        )
        
        result = self.generate_text(surveillance_prompt, max_length=200, temperature=0.5)
        
        if "error" in result:
            return result
        
        # Also classify the threat level
        threat_categories = ["no threat", "low threat", "medium threat", "high threat"]
        threat_analysis = self.classify_text(image_description, threat_categories)
        
        return {
            "surveillance_analysis": result["generated_text"],
            "threat_classification": threat_analysis.get("classification", "unknown"),
            "image_description": image_description,
            "question": question
        }
    
    def clear_memory(self):
        """Clear GPU memory"""
        if self.device.type == 'cuda':
            torch.cuda.empty_cache()
        gc.collect()
    
    def get_model_info(self):
        """
        Get model information
        
        Returns:
            dict: Model information
        """
        return {
            "model_name": self.model_name,
            "device": str(self.device),
            "loaded": self.loaded,
            "cuda_available": torch.cuda.is_available(),
            "gpu_memory": torch.cuda.get_device_properties(0).total_memory if torch.cuda.is_available() else None
        }
