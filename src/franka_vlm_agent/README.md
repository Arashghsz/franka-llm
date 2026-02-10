# VLM Agent - Vision Language Model for Franka

**Status**: ✅ Ready for deployment on Jetson AGX Orin  
**Model**: LLaVA 7B via Ollama  
**Purpose**: Semantic scene understanding of table objects

## What's Included

### Core Components
- **`vlm_node.py`**: Main ROS 2 node that subscribes to camera images and publishes VLM explanations
- **`vlm_test.py`**: Testing utility to verify Ollama and model setup
- **`vlm.launch.py`**: Launch file with configurable parameters
- **`vlm_image_analyzer.py`**: Alternative image analysis implementation

### Setup & Configuration
- **`VLM_SETUP.md`**: Comprehensive setup guide (read this first!)
- **`setup_ollama_jetson.sh`**: Automated Ollama + LLaVA installation
- **`quick_start_vlm.sh`**: One-command quick start script
- **`package.xml`**: ROS 2 package metadata
- **`setup.py`**: Python package configuration with entry points

## Quick Start

### Option 1: Automated Setup (Recommended)
```bash
cd /home/arash/franka-llm/src/franka_vlm_agent
bash quick_start_vlm.sh
```

### Option 2: Manual Setup
```bash
# Install Ollama and LLaVA 7B
bash setup_ollama_jetson.sh

# Build the package
cd /home/arash/franka-llm
colcon build --packages-select franka_vlm_agent
source install/setup.bash

# Test the setup
ros2 run franka_vlm_agent vlm_test

# Launch the VLM node
ros2 launch franka_vlm_agent vlm.launch.py
```

## System Architecture

```
PC Controller (RealSense Camera)
    ↓ publishes /camera/color/image_raw
    ↓ ROS 2 Network (DDS)
    ↓
Jetson AGX Orin (VLM Node)
    ↓ receives images
    ↓ sends to Ollama/LLaVA 7B
    ↓ publishes /vlm/explanation (text)
    ↓ publishes /vlm/status (health)
```

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | IN | Camera images from PC |
| `/vlm/explanation` | `std_msgs/String` | OUT | VLM's description of table contents |
| `/vlm/status` | `std_msgs/String` | OUT | Health/status messages |

## Configuration Parameters

Launch with custom parameters:
```bash
ros2 launch franka_vlm_agent vlm.launch.py \
  camera_topic:=/camera/rgb/image \
  analysis_rate:=0.5 \
  vlm_model:=llava:13b \
  debug:=True
```

See `VLM_SETUP.md` for full parameter documentation.

## Troubleshooting

### "Cannot connect to Ollama"
```bash
# Start Ollama
ollama serve &

# Verify
curl http://localhost:11434/api/tags
```

### "Model not found"
```bash
# Pull the model
ollama pull llava:7b

# List available
ollama list
```

### No camera images
- Verify ROS_DOMAIN_ID is set on both PC and Jetson
- Check PC is publishing camera: `ros2 topic list | grep camera`
- Check network connectivity between machines

## Performance

- **Latency**: 5-15 seconds per image (network + model inference)
- **Memory**: ~6GB VRAM (LLaVA 7B)
- **Throughput**: 1 image per second by default (configurable)

## Integration with System

The VLM explanation is used by:
1. **YOLO Detector**: Filter detections based on VLM semantic understanding
2. **Coordinator**: Make decisions based on scene understanding
3. **Motion Executor**: Adjust grasp strategy based on object descriptions

See [THESIS_GOALS.md](../../../THESIS_GOALS.md) for system architecture.

## Next Steps

- [ ] Integrate with YOLO detector on PC controller
- [ ] Connect to motion executor for object grasping
- [ ] Log all VLM explanations for evaluation
- [ ] Test with multiple objects on table
- [ ] Optimize latency for real-time performance

## References

- **Ollama**: https://ollama.ai (Model serving)
- **LLaVA**: https://github.com/haotian-liu/LLaVA (Vision Language Model)
- **ROS 2**: https://docs.ros.org/en/humble/ (Middleware)
- **Full Setup Guide**: See `VLM_SETUP.md` in this directory

---

**Maintainer**: Arash  
**Last Updated**: February 2026
