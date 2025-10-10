#!/usr/bin/env python3
"""
Complete usage example for BLIP2 service with ROS2 integration
"""

import sys
import os
from pathlib import Path
import time

# Add package to path
PACKAGE_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PACKAGE_ROOT))

def main():
    """Main function demonstrating BLIP2 usage"""
    
    print("🤖 BLIP2 Complete Usage Example")
    print("=" * 40)
    
    try:
        from vlm_inspection.blip2_service import BLIP2Service
        from PIL import Image as PILImage
        import torch
        
        print(f"✓ Dependencies loaded (CUDA: {torch.cuda.is_available()})")
        
        # Initialize service with different models
        models_to_try = [
            "Salesforce/blip2-flan-t5-xl",  # Good balance of performance and speed
            "Salesforce/blip2-opt-2.7b"     # Faster, smaller model
        ]
        
        for i, model_name in enumerate(models_to_try):
            print(f"\n--- Testing Model {i+1}: {model_name} ---")
            
            # Initialize service
            blip2 = BLIP2Service(model_name)
            
            # Load model
            print("📥 Loading model...")
            start_time = time.time()
            
            if not blip2.load_model():
                print(f"❌ Failed to load {model_name}")
                continue
                
            load_time = time.time() - start_time
            print(f"✅ Model loaded in {load_time:.2f} seconds")
            
            # Test with sample image
            assets_dir = PACKAGE_ROOT / "assets"
            sample_images = list(assets_dir.glob("*.jpg"))
            
            if sample_images:
                test_image = sample_images[0]
                print(f"📸 Testing with: {test_image.name}")
                
                # Basic image description
                print("\n1. Basic Image Description:")
                result = blip2.describe_image_from_path(str(test_image))
                if "error" not in result:
                    print(f"📝 {result['result']}")
                else:
                    print(f"❌ {result['error']}")
                
                # Question answering
                print("\n2. Question Answering:")
                questions = [
                    "What do you see?",
                    "What colors are present?",
                    "How many objects are visible?"
                ]
                
                for question in questions:
                    print(f"❓ {question}")
                    result = blip2.answer_question_from_path(str(test_image), question)
                    if "error" not in result:
                        print(f"💡 {result['result']}")
                    else:
                        print(f"❌ {result['error']}")
                
                # Guided description with prompt
                print("\n3. Guided Description:")
                prompt = "Describe this image in great detail, including colors, objects, and their positions:"
                result = blip2.describe_image_from_path(str(test_image), prompt)
                if "error" not in result:
                    print(f"📝 {result['result']}")
                else:
                    print(f"❌ {result['error']}")
            
            print(f"\n--- Model {model_name} testing complete ---")
            
            # Only test one model for quick demo
            break
        
        print("\n🎯 BLIP2 Service Features:")
        print("• Image description generation")
        print("• Visual question answering") 
        print("• Guided description with prompts")
        print("• Multiple model support")
        print("• GPU acceleration when available")
        
        print("\n📚 Available Models:")
        blip2_service = BLIP2Service()
        for model in blip2_service.get_available_models():
            print(f"• {model}")
        
        print("\n🔌 ROS2 Integration:")
        print("• Launch: ros2 launch vlm_inspection blip2_launch.py")
        print("• Image topic: /camera/image_raw")
        print("• Description topic: /blip2/description")
        print("• Question topic: /blip2/question")
        print("• Answer topic: /blip2/answer")
        
        print("\n✅ Example completed successfully!")
        
    except ImportError as e:
        print(f"❌ Missing dependencies: {e}")
        print("Install with: pip install transformers torch pillow")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
