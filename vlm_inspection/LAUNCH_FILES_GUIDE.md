# VLM Surveillance Launch Files Guide

This document provides usage examples for all VLM surveillance launch files.

## Available Launch Files

### 1. `nanovlm_surveillance_launch.py`
Ultra-efficient surveillance using nanoVLM (~222M parameters)

### 2. `smolvlm_surveillance_launch.py`  
Advanced surveillance using SmolVLM (256M/500M parameters)

### 3. `surveillance_launch.py`
Original Florence-2 based surveillance

### 4. `qwen2_vl_surveillance_launch.py`
Advanced multilingual surveillance using Qwen2-VL (2B/7B parameters)

### 5. `phi35_vision_surveillance_launch.py`
State-of-the-art surveillance using Phi-3.5-vision-instruct (4.2B parameters)

### 6. `multi_vlm_surveillance_launch.py`
Comprehensive launch file for running multiple VLM nodes simultaneously

## Usage Examples

### Launch nanoVLM Surveillance

**Basic Launch:**
```bash
ros2 launch vlm_inspection nanovlm_surveillance_launch.py
```

**With Custom Parameters:**
```bash
ros2 launch vlm_inspection nanovlm_surveillance_launch.py \
  input_topic:=/camera/image_raw \
  analysis_rate:=3.0 \
  enable_vqa:=true
```

**Parameters:**
- `input_topic`: Image topic (default: `/quadrotor_1/slot0/image_raw`)
- `model_name`: nanoVLM model (default: `microsoft/nanoVLM`)
- `analysis_rate`: Analysis frequency in Hz (default: `2.0`)
- `enable_vqa`: Enable Visual Question Answering (default: `true`)

### Launch SmolVLM Surveillance

**Basic Launch:**
```bash
ros2 launch vlm_inspection smolvlm_surveillance_launch.py
```

**With 500M Model:**
```bash
ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
  model_name:=HuggingFaceTB/SmolVLM-500M-Instruct \
  analysis_rate:=1.0
```

**Full Configuration:**
```bash
ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
  input_topic:=/camera/image_raw \
  model_name:=HuggingFaceTB/SmolVLM-256M-Instruct \
  analysis_rate:=1.5 \
  enable_compliance_check:=true \
  enable_detailed_vqa:=true
```

**Parameters:**
- `input_topic`: Image topic (default: `/quadrotor_1/slot0/image_raw`)
- `model_name`: SmolVLM variant (default: `HuggingFaceTB/SmolVLM-256M-Instruct`)
- `analysis_rate`: Analysis frequency in Hz (default: `1.5`)
- `enable_compliance_check`: Enable regulatory compliance (default: `true`)
- `enable_detailed_vqa`: Enable detailed VQA (default: `true`)

### Launch Qwen2-VL Surveillance

**Basic Launch:**
```bash
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py
```

**With Custom Model Variant:**
```bash
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
  model_variant:=Qwen/Qwen2-VL-7B-Instruct \
  analysis_rate:=0.8
```

**Multilingual Configuration:**
```bash
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
  language_mode:=bilingual \
  threat_sensitivity:=high \
  input_topic:=/camera/image_raw
```

**Full Advanced Configuration:**
```bash
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
  model_variant:=Qwen/Qwen2.5-VL-3B-Instruct \
  language_mode:=chinese \
  threat_sensitivity:=high \
  surveillance_frequency:=1.5 \
  use_advanced_analysis:=true \
  compliance_monitoring:=true \
  advanced_vqa:=true
```

**Parameters:**
- `model_variant`: Qwen2-VL model (default: `Qwen/Qwen2-VL-2B-Instruct`)
  - Available: `Qwen/Qwen2-VL-2B-Instruct`, `Qwen/Qwen2-VL-7B-Instruct`, `Qwen/Qwen2.5-VL-2B-Instruct`, `Qwen/Qwen2.5-VL-3B-Instruct`, `Qwen/Qwen2.5-VL-7B-Instruct`
- `language_mode`: Language support (default: `english`)
  - Options: `english`, `chinese`, `bilingual`
- `threat_sensitivity`: Detection sensitivity (default: `medium`)
- `surveillance_frequency`: Analysis frequency in Hz (default: `1.0`)
- `input_topic`: Image topic (default: `/quadrotor_1/slot0/image_raw`)
- `use_advanced_analysis`: Enable multi-perspective scene understanding (default: `true`)
- `compliance_monitoring`: Enable regulatory compliance (default: `true`)
- `advanced_vqa`: Enable advanced visual question answering (default: `true`)

### Launch Phi-3.5 Vision Surveillance

**Basic Launch:**
```bash
ros2 launch vlm_inspection phi35_vision_surveillance_launch.py
```

**High-Performance Configuration:**
```bash
ros2 launch vlm_inspection phi35_vision_surveillance_launch.py \
  threat_sensitivity:=high \
  surveillance_frequency:=2.0 \
  use_advanced_analysis:=true \
  compliance_monitoring:=true
```

**Parameters:**
- `threat_sensitivity`: Detection sensitivity (default: `medium`)
- `surveillance_frequency`: Analysis frequency in Hz (default: `2.0`)
- `use_advanced_analysis`: Enable multi-task scene understanding (default: `true`)
- `compliance_monitoring`: Enable regulatory compliance (default: `true`)

### Launch Multiple VLM Surveillance Nodes

**Launch Qwen2-VL and SmolVLM Together:**
```bash
ros2 launch vlm_inspection multi_vlm_surveillance_launch.py \
  enable_qwen2_vl:=true \
  enable_smolvlm:=true \
  enable_florence:=false \
  enable_blip2:=false
```

**Launch All VLM Nodes for Comparison:**
```bash
ros2 launch vlm_inspection multi_vlm_surveillance_launch.py \
  enable_florence:=true \
  enable_blip2:=true \
  enable_nanovlm:=true \
  enable_smolvlm:=true \
  enable_qwen2_vl:=true \
  enable_phi35_vision:=true \
  florence_analysis_rate:=0.5 \
  blip2_analysis_rate:=0.3 \
  nanovlm_analysis_rate:=2.0 \
  smolvlm_analysis_rate:=1.5 \
  qwen2_vl_analysis_rate:=1.0 \
  phi35_vision_analysis_rate:=1.5
```

**Multilingual Surveillance Setup:**
```bash
ros2 launch vlm_inspection multi_vlm_surveillance_launch.py \
  enable_qwen2_vl:=true \
  enable_phi35_vision:=true \
  qwen2_vl_language_mode:=bilingual \
  qwen2_vl_model:=Qwen/Qwen2.5-VL-3B-Instruct \
  phi35_vision_threat_sensitivity:=high
```

**Custom Model Configuration:**
```bash
ros2 launch vlm_inspection multi_vlm_surveillance_launch.py \
  enable_nanovlm:=true \
  enable_smolvlm:=true \
  enable_qwen2_vl:=true \
  nanovlm_model:=microsoft/nanoVLM \
  smolvlm_model:=HuggingFaceTB/SmolVLM-500M-Instruct \
  qwen2_vl_model:=Qwen/Qwen2-VL-7B-Instruct \
  qwen2_vl_language_mode:=english \
  input_topic:=/my_camera/image_raw
```

## Performance Optimization Guidelines

### Resource Management
When running multiple surveillance nodes:

1. **Memory Usage**: Start with smaller models first
   ```bash
   # Start nanoVLM first (lowest memory)
   ros2 launch vlm_inspection nanovlm_surveillance_launch.py &
   
   # Then SmolVLM-256M
   ros2 launch vlm_inspection smolvlm_surveillance_launch.py &
   ```

2. **Analysis Rate Tuning**: Adjust based on system capabilities
   ```bash
   # Conservative rates for multiple nodes
   ros2 launch vlm_inspection multi_vlm_surveillance_launch.py \
     nanovlm_analysis_rate:=1.0 \
     smolvlm_analysis_rate:=0.8
   ```

### GPU vs CPU Deployment

**GPU Deployment (Recommended):**
```bash
# Export CUDA device
export CUDA_VISIBLE_DEVICES=0

# Launch with higher analysis rates
ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
  analysis_rate:=2.0
```

**CPU Deployment:**
```bash
# Use nanoVLM for CPU efficiency
ros2 launch vlm_inspection nanovlm_surveillance_launch.py \
  analysis_rate:=1.0
```

## Monitoring and Debugging

### Check Node Status
```bash
# List running surveillance nodes
ros2 node list | grep surveillance

# Check specific node info
ros2 node info /nanovlm/nanovlm_surveillance_node
ros2 node info /smolvlm/smolvlm_surveillance_node
```

### Monitor Resources
```bash
# Monitor CPU/Memory usage
htop

# Monitor GPU usage (if available)
nvidia-smi -l 1
```

### View Surveillance Outputs
```bash
# Monitor logs in real-time
ros2 launch vlm_inspection nanovlm_surveillance_launch.py | tee surveillance_log.txt

# Check specific node output
ros2 topic echo /surveillance_analysis
```

## Troubleshooting

### Common Issues and Solutions

1. **Model Loading Errors**
   ```bash
   # Check available disk space
   df -h
   
   # Test model loading independently
   python3 -c "from vlm_inspection.nanovlm_service import NanoVLMService; s=NanoVLMService(); print(s.load_model())"
   ```

2. **Image Topic Issues**
   ```bash
   # List available image topics
   ros2 topic list | grep image
   
   # Check image message format
   ros2 topic info /quadrotor_1/slot0/image_raw
   ```

3. **Performance Issues**
   ```bash
   # Reduce analysis rate for Qwen2-VL
   ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
     surveillance_frequency:=0.5
   
   # Use smaller Qwen2-VL model
   ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
     model_variant:=Qwen/Qwen2-VL-2B-Instruct
   
   # Use smaller SmolVLM model
   ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
     model_name:=HuggingFaceTB/SmolVLM-256M-Instruct
   ```

## Integration with Existing Systems

### Maritime Port Surveillance
```bash
# Configure Qwen2-VL for port monitoring with Chinese support
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
  input_topic:=/port_camera/image_raw \
  language_mode:=bilingual \
  compliance_monitoring:=true \
  surveillance_frequency:=1.0

# Configure SmolVLM for port monitoring
ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
  input_topic:=/port_camera/image_raw \
  enable_compliance_check:=true \
  analysis_rate:=1.0
```

### UAV Surveillance
```bash
# Configure for drone surveillance
ros2 launch vlm_inspection nanovlm_surveillance_launch.py \
  input_topic:=/uav/camera/image_raw \
  analysis_rate:=2.0 \
  enable_vqa:=true
```

### Multi-Camera Setup
```bash
# Launch multiple instances for different cameras with different models
ros2 launch vlm_inspection qwen2_vl_surveillance_launch.py \
  input_topic:=/camera1/image_raw \
  language_mode:=english &

ros2 launch vlm_inspection smolvlm_surveillance_launch.py \
  input_topic:=/camera2/image_raw &

ros2 launch vlm_inspection phi35_vision_surveillance_launch.py \
  input_topic:=/camera3/image_raw &
```

## Model Comparison and Selection Guide

### Model Performance Characteristics

| Model | Parameters | Memory Usage | Speed | Language Support | Best Use Case |
|-------|------------|--------------|-------|------------------|---------------|
| nanoVLM | ~222M | ~2GB | Fastest | English | Real-time monitoring |
| SmolVLM-256M | 256M | ~2GB | Fast | English | Balanced performance |
| SmolVLM-500M | 500M | ~3GB | Medium-Fast | English | Enhanced accuracy |
| Qwen2-VL-2B | 2B | ~4GB | Fast | English/Chinese | Multilingual surveillance |
| Qwen2-VL-7B | 7B | ~14GB | Medium | English/Chinese | High-accuracy analysis |
| Qwen2.5-VL-3B | 3B | ~6GB | Medium-Fast | English/Chinese | Latest technology |
| Phi-3.5-Vision | 4.2B | ~8GB | Medium | English | State-of-the-art reasoning |

### Selection Guidelines

**For Real-time Applications:**
- Use nanoVLM or SmolVLM-256M
- Analysis rate: 2.0+ Hz

**For Multilingual Requirements:**
- Use Qwen2-VL variants
- Supports English, Chinese, and bilingual modes

**For Highest Accuracy:**
- Use Phi-3.5-Vision or Qwen2-VL-7B
- Lower analysis rate recommended (1.0 Hz)

**For Resource-Constrained Environments:**
- Use nanoVLM or SmolVLM-256M
- CPU deployment supported
