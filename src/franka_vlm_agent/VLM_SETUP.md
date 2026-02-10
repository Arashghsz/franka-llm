# VLM Agent Setup Guide - LLaVA 7B on Jetson

This guide walks you through setting up the Vision Language Model (VLM) agent using LLaVA 7B on Jetson AGX Orin.

## Architecture Overview

```
┌────────────────────────────────────────────────────────────┐
│           PC CONTROLLER (RealSense Camera)                  │
│  Publishes: /camera/color/image_raw                         │
└────────────────────┬───────────────────────────────────────┘
                     │ ROS 2 Network
                     ↓
┌────────────────────────────────────────────────────────────┐
│              JETSON AGX ORIN (VLM Node)                     │
│  Subscribes: /camera/color/image_raw                        │
│  Processes:  LLaVA 7B via Ollama                            │
│  Publishes:  /vlm/explanation (text analysis)               │
│  Publishes:  /vlm/status (health/debug info)                │
└────────────────────────────────────────────────────────────┘
```

## Prerequisites

### Hardware
- **Jetson AGX Orin** (or similar ARM64 GPU)
- **RealSense camera** on PC controller
- **Network connection** between Jetson and PC (ROS 2 domain)

### Software
- **ROS 2** (Humble or newer)
- **Ollama** (for LLaVA model serving)
- **Python 3.10+**

---

## Installation Steps

### Step 1: Install Ollama on Jetson

```bash
# Download and install Ollama for ARM64
curl -fsSL https://ollama.ai/install.sh | sh

# Verify installation
ollama --version
```

### Step 2: Pull LLaVA 7B Model

This will download the ~5GB LLaVA 7B model:

```bash
# Start Ollama server
ollama serve &

# Pull the model (takes several minutes)
ollama pull llava:7b

# Verify it's available
ollama list
```

The first time you pull, this may take 5-10 minutes. Ollama will cache the model locally.

### Step 3: Build the VLM Package

```bash
# From workspace root
cd /home/arash/franka-llm

# Build just the VLM package
colcon build --packages-select franka_vlm_agent

# Source the environment
source install/setup.bash
```

### Step 4: Test Ollama Connection

```bash
# Test if Ollama is working with a sample image
ros2 run franka_vlm_agent vlm_test

# Or with a specific image:
ros2 run franka_vlm_agent vlm_test --image /path/to/image.jpg

# Or with a custom Ollama host:
ros2 run franka_vlm_agent vlm_test --ollama-host http://localhost:11434 --model llava:7b
```

---

## Running the VLM Node

### Option 1: Launch with Default Settings

```bash
# On Jetson
source install/setup.bash
ros2 launch franka_vlm_agent vlm.launch.py
```

This will:
- Subscribe to `/camera/color/image_raw` from PC controller
- Use `http://localhost:11434` as Ollama host
- Analyze images at 1 Hz (once per second)
- Publish explanations to `/vlm/explanation`

### Option 2: Launch with Custom Parameters

```bash
# Specify custom camera topic and analysis rate
ros2 launch franka_vlm_agent vlm.launch.py \
  camera_topic:=/camera/rgb/image \
  analysis_rate:=0.5 \
  debug:=True

# Or use a remote Ollama server
ros2 launch franka_vlm_agent vlm.launch.py \
  ollama_host:=http://192.168.1.100:11434
```

### Available Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `/camera/color/image_raw` | Camera topic from PC |
| `ollama_host` | `http://localhost:11434` | Ollama server address |
| `vlm_model` | `llava:7b` | Model to use (`llava:7b`, `llava:13b`, `vila`, etc.) |
| `analysis_rate` | `1.0` | Analysis frequency (Hz) |
| `timeout` | `30` | VLM response timeout (seconds) |
| `debug` | `False` | Enable debug logging |

---

## PC Controller Setup

On the PC with RealSense camera, you need to:

1. **Have ROS 2 installed and configured**
2. **Set ROS_DOMAIN_ID to match Jetson** (e.g., `export ROS_DOMAIN_ID=0`)
3. **Publish camera images**:

```bash
# Option A: Use realsense-ros driver
ros2 launch realsense2_camera rs_launch.py

# Option B: Use your custom camera node
python3 src/franka_vision_detection/franka_vision_detection/camera_publisher.py
```

### Verify Network Connection

From Jetson, check if you can see PC topics:

```bash
# List all available topics (should see /camera/color/image_raw)
ros2 topic list

# Monitor camera images
ros2 topic hz /camera/color/image_raw
```

---

## Testing the Setup

### Test 1: Check Ollama Health

```bash
# On Jetson
ros2 run franka_vlm_agent vlm_test --ollama-host http://localhost:11434
```

Expected output:
```
✓ Ollama is running at http://localhost:11434
✓ Available models:
  - llava:7b
```

### Test 2: Check Camera Feed

```bash
# On PC, verify camera is publishing
ros2 topic hz /camera/color/image_raw

# Should show something like:
# average rate: 30.123 Hz
```

### Test 3: Run VLM Node with Debug

```bash
# On Jetson, launch with debug enabled
ros2 launch franka_vlm_agent vlm.launch.py debug:=True

# Watch the output:
# [vlm_node-1] Analyzing table scene with VLM...
# [vlm_node-1] Published explanation: On the table I can see...
```

### Test 4: Monitor Output Topic

```bash
# Open another terminal and listen to VLM output
ros2 topic echo /vlm/explanation

# Watch status updates
ros2 topic echo /vlm/status
```

---

## Troubleshooting

### Problem: "Cannot connect to Ollama"

**Solution:**
```bash
# Make sure Ollama is running
ps aux | grep ollama

# If not running, start it
ollama serve &

# Test local connection
curl http://localhost:11434/api/tags
```

### Problem: "Model not found"

**Solution:**
```bash
# List available models
ollama list

# Pull the model if missing
ollama pull llava:7b

# Or use a different model you have:
ros2 launch franka_vlm_agent vlm.launch.py vlm_model:=llava:13b
```

### Problem: No camera images on Jetson

**Solution:**
```bash
# On Jetson, verify network
ros2 topic list | grep camera

# If no camera topics, check on PC:
# 1. Is ROS_DOMAIN_ID set correctly on both machines?
export ROS_DOMAIN_ID=0  # Same on both!

# 2. Is camera publisher running on PC?
ros2 topic hz /camera/color/image_raw

# 3. Check firewall on PC (port 4317 and others may need opening)
sudo ufw allow from 192.168.1.0/24  # Your network range
```

### Problem: VLM is very slow (> 30s per image)

**Solution:**
1. Reduce image quality or size
2. Use smaller model: `vlm_model:=llava:7b` (instead of 13b)
3. Reduce analysis rate: `analysis_rate:=0.2` (process every 5s instead of 1s)
4. Check Jetson GPU usage: `tegrastats`

---

## Output Topics

### `/vlm/explanation` (std_msgs/String)
The VLM's description of the table scene.

Example:
```
On the table I can see:
- A red apple on the left side
- A green cube in the center
- A blue cup on the right
- The table has a wooden surface
```

### `/vlm/status` (std_msgs/String)
Health and status messages.

Examples:
```
ANALYSIS_COMPLETE: 14:23:45
ERROR: Could not get VLM response
ANALYSIS_FAILED: Timeout waiting for model
```

---

## Advanced Configuration

### Use Different VLM Models

```bash
# LLaVA 13B (slower but more accurate)
ros2 launch franka_vlm_agent vlm.launch.py vlm_model:=llava:13b

# VILA model (if available)
ollama pull vila
ros2 launch franka_vlm_agent vlm.launch.py vlm_model:=vila

# Check what's available
ollama list
```

### Integration with Other Nodes

Connect VLM output to YOLO detection:

```python
# In your coordinator node
sub = node.create_subscription(
    String,
    '/vlm/explanation',
    vlm_callback,
    10
)

def vlm_callback(msg: String):
    explanation = msg.data
    # Use explanation to guide YOLO detection
    trigger_yolo_search(explanation)
```

### Logging Explanations

```bash
# Record all VLM outputs to a bag
ros2 bag record /vlm/explanation /vlm/status -o vlm_data.db3

# Play back later for analysis
ros2 bag play vlm_data.db3
```

---

## Performance Notes

- **Latency**: ~5-15 seconds per image (depends on model and Jetson load)
- **Memory**: LLaVA 7B uses ~6GB VRAM
- **Throughput**: At 1 Hz analysis rate, CPU load is low
- **Optimization**: Reduce `analysis_rate` to 0.2-0.5 Hz for real-time performance

---

## Next Steps

1. ✅ Set up Ollama + LLaVA 7B
2. ✅ Launch VLM node on Jetson
3. ⏭️ Integrate with YOLO detector on PC for object localization
4. ⏭️ Connect to motion executor for pick & place

See the main [README.md](../../../README.md) and [todo.md](../../../todo.md) for full system integration.

---

## References

- **Ollama Docs**: https://ollama.ai
- **LLaVA Project**: https://github.com/haotian-liu/LLaVA
- **ROS 2 Network Setup**: https://docs.ros.org/en/humble/Concepts/Advanced/Middleware-Implementation-Guide.html
