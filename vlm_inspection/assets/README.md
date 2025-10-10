# Sample Images for VLM Inspection Package

This directory contains sample images for testing the Florence-2 integration.

## Available Images

### cat_sample.jpg
- **Description**: A cat image for testing animal recognition
- **Use case**: Good for testing detailed descriptions and object recognition
- **Source**: Wikimedia Commons

### sample_image.jpg  
- **Description**: PNG transparency demonstration image
- **Use case**: Testing with different image formats and transparency
- **Source**: Wikimedia Commons

### test_image.jpg
- **Description**: Random photograph from Lorem Picsum
- **Use case**: General testing with varied content
- **Source**: Lorem Picsum (random image service)

## Usage Examples

### Basic Testing
```bash
# Test with the cat image
ros2 run vlm_inspection image_describer assets/cat_sample.jpg

# Test with different tasks
ros2 run vlm_inspection image_describer assets/cat_sample.jpg --task "<CAPTION>"
ros2 run vlm_inspection image_describer assets/cat_sample.jpg --task "<OD>"
```

### ROS2 Node Testing
```bash
# Terminal 1: Start Florence node
ros2 launch vlm_inspection florence_launch.py

# Terminal 2: Publish test image
python3 examples/image_publisher.py assets/cat_sample.jpg

# Terminal 3: Listen to descriptions
ros2 topic echo /florence/description
```

## Adding Your Own Images

You can add your own test images to this directory:

1. Copy images to this assets folder
2. Use common formats: `.jpg`, `.png`, `.jpeg`
3. Recommended resolution: 400x300 to 800x600 pixels
4. Test with: `ros2 run vlm_inspection image_describer assets/your_image.jpg`

## Florence-2 Tasks to Try

Different tasks work better with different types of images:

- **`<MORE_DETAILED_CAPTION>`**: Works well with all images (default)
- **`<OD>`**: Best with images containing clear objects (try with cat_sample.jpg)
- **`<OCR>`**: Use with images containing text
- **`<DENSE_REGION_CAPTION>`**: Good for complex scenes with multiple objects

## Tips

- **First run**: Model download takes time (2-3 GB for Florence-2-large)
- **GPU**: Much faster with CUDA-capable GPU
- **Memory**: Ensure at least 4GB RAM available
- **Internet**: Required for initial model download

## Troubleshooting

If you encounter issues:

1. **Image not found**: Use absolute paths or run from workspace root
2. **Model download fails**: Check internet connection and disk space
3. **Out of memory**: Try Florence-2-base instead of Florence-2-large
4. **Slow inference**: Check if CUDA is properly installed for GPU acceleration
