# Quick Start: VLM Scene Analysis Pipeline

## Configuration

All models and settings are configured in **`config.yaml`** at the repo root. Edit this file to change:

- **LLM Model**: `llm.model` (default: `llama3.1:8b`)
  - Options: `llama3.1:8b`, `llama3.1:70b`, `llama3.2:3b`, `mistral`, `phi3`, etc.
- **VLM Model**: `vlm.model` (default: `llava:7b`)
  - Options: `llava:7b`, `llava:13b`, `llava:34b`, `bakllava`, `llava-phi3`, etc.
- **Ollama URLs**: `llm.ollama_url` and `vlm.ollama_url` (default: `http://localhost:11434`)
- **Timeouts**: `llm.timeout` (60s) and `vlm.timeout` (30s)
- **Camera Topics**: `camera.topic`, `camera.depth_topic`, etc.
- **Debug Options**: `debug.vlm_debug`, `debug.llm_debug`, `debug.log_level`

**Example - Switching to Llama 3.2 and LLaVA 13B:**
```yaml
llm:
  model: "llama3.2:3b"
vlm:
  model: "llava:13b"
```

No rebuild needed - just restart the nodes!

## Architecture Flow

```
User Command → LLM Coordinator (routing + conversation) → VLM (scene analysis + object center depth) OR Motion
```

**Key Concepts:**
- **LLM Coordinator**: Conversational intelligent router that greets users and routes to appropriate agent
- **VLM Agent**: Analyzes scene, describes objects, **finds object centers** and provides 3D position at that center
- **Object Center Depth**: When user asks for specific object, VLM finds it and returns its center pixel + depth
- **No YOLO, no bounding boxes** - just object center point localization

## Terminal Setup (3 terminals)

### Terminal 1: RealSense Cameras
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 launch realsense_cameras cameras.launch.py
```

### Terminal 2: LLM Coordinator (Intelligent Router)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_llm_planner llm_coordinator
# Routes commands to appropriate agents using LLM decision-making
```

### Terminal 3: VLM Agent (Scene Analysis + Depth)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_vlm_agent vlm_node
# Analyzes scene and provides center pixel 3D position
# Uses EE camera compressed images by default
```

### Terminal 4: Interactive Chat
```bash
cd ~/franka-llm
source install/setup.zsh
python3 src/chat_coordinator.py
```

## Example Usage

### Greeting the Robot
```
> hello
[Coordinator] Hello! I'm Franka Assistant, your robot coordinator. I can help you 
              inspect the workspace or manipulate objects. What would you like to do?
```

### Asking Who It Is
```
> who are you?
[Coordinator] I'm Franka Assistant, an intelligent coordinator for the Franka robot 
              system. I work with a vision system and motion planner to help you 
              interact with the workspace.
```

### Scene Inspection
```
> what do you see at the table?
[Coordinator] Routing to VLM agent...
[VLM] I can see several objects on the table including a red plastic cup 
      on the left side, a green apple in the center, and a screwdriver 
      near the right edge. The table surface appears to be light colored.
[3D Position] Center of scene (image center): X: 0.523m, Y: 0.142m, Z: 0.650m
```

### Object Localization (Finding Object Center)
```
> where is the red cup?
[Coordinator] Routing to VLM agent...
[Coordinator] Requesting VLM to locate: red cup
[VLM] The red cup is located on the left side of the table
[3D Position] Object center at pixel (180, 240): X: -0.125m, Y: 0.085m, Z: 0.620m
```

### Manipulation Intent (Future)
```
> pick up the apple
[Coordinator] Routing to both VLM and Motion agents...
[Coordinator] Requesting VLM to locate: apple
[VLM] I found a green apple in the center of the workspace
[3D Position] Object center at pixel (320, 240): X: 0.005m, Y: -0.012m, Z: 0.680m
[Motion] Planning pick motion to target position...
```

## Topics Reference

| Topic | Type | Purpose |
|-------|------|---------|
| `/user_command` | String | User input to LLM Coordinator |
| `/coordinator_response` | String | Coordinator responses back to user |
| `/vlm_request` | String | Request to VLM for scene analysis |
| `/vlm/explanation` | String | VLM scene description |
| `/vlm_center_position` | PoseStamped | 3D coordinates of center pixel in camera frame |
| `/motion_command` | String | Commands to motion executor |

## Testing Components

### Test Center Pixel Depth Sampling
Test depth resolution before running full pipeline:

```bash
# Terminal 1: Cameras (as above)
# Terminal 2: Test depth sampling
cd ~/franka-llm
source install/setup.zsh
python3 src/test_depth_center_pixel.py

# This will continuously sample and display:
# - Center pixel coordinates
# - Depth at center
# - 3D position in camera frame
```

### Test VLM Without Coordinator
```bash
# Terminal 1-2: Cameras + VLM (as above)
# Terminal 3: Manual VLM request
ros2 topic pub --once /vlm_request std_msgs/String \
  'data: "{\"type\": \"scene_description\"}"'

# Listen for response
ros2 topic echo /vlm/explanation
ros2 topic echo /vlm_center_position
```

## Current State

✅ LLM Coordinator is conversational (greets, introduces itself)
✅ VLM locates **object centers** (not just image center)
✅ VLM provides 3D position of target object's center pixel
✅ Depth resolution integrated into VLM agent
✅ Coordinator uses VLM center pixel directly (no bbox recalculation) - **Updated Feb 19, 2026**
✅ Camera→robot TF transform configured
✅ Simple, clean architecture with clear separation of concerns
⚠️ Motion execution not yet fully tested
⚠️ Hand-eye calibration may need adjustment for accurate positioning

### Camera Transform
The camera-to-robot transform is configured as:
```bash
ros2 run tf2_ros static_transform_publisher \
  0.0458961 -0.0368559 0.0567165 \
  -0.000400857 -0.00425145 0.698275 0.715817 \
  fr3_hand ee_d435i_color_optical_frame
```
- **Translation**: [0.046m, -0.037m, 0.057m]
- **Rotation**: Quaternion [-0.0004, -0.004, 0.698, 0.716]
- **Recalibrate**: Use `python3 calibrate_hand_eye.py` if needed

## Topics Reference

| Topic | Type | Purpose |
|-------|------|---------|
| `/user_command` | String | User input to coordinator |
| `/coordinator_response` | String | Coordinator responses (greetings, confirmations) |
| `/vlm_request` | String | Request to VLM (scene or locate object) |
| `/vlm/explanation` | String | VLM scene description |
| `/vlm_center_position` | PoseStamped | 3D coordinates of **object center** |
| `/motion_command` | String | Commands to motion planner |

## Architecture Benefits

**Object-Centric:**
- VLM finds the **target object's center**, not just image center
- Depth is sampled at the object's location
- More accurate for manipulation tasks

**Conversational:**
- Coordinator greets users and introduces itself
- Handles small talk naturally
- Routes task commands intelligently

**Clear Responsibilities:**
- **LLM Coordinator**: Conversation + routing based on intent
- **VLM Agent**: Object localization + center point depth
- **Motion Agent**: Execute physical movements (future)

## Next Steps

1. **Test the new architecture** - Run test_depth_center_pixel.py first
2. **Verify calibration** - If positions are inaccurate, run `python3 calibrate_hand_eye.py`
3. **Change models** - Edit `config.yaml` to try different LLM/VLM models
   - Pull new models: `ollama pull llama3.2:3b` or `ollama pull llava:13b`
4. **Motion integration** - Test full pick/place pipeline (see test_full_pick.md)
5. **Better VLM models** - Try Qwen-VL or Florence-2 for improved scene understanding
6. **Multi-point sampling** - Extend beyond center pixel if needed

## Configuration Reference

### Available LLM Models (via Ollama)
- `llama3.1:8b` - Default, good balance (4.7 GB)
- `llama3.2:3b` - Faster, smaller (2 GB)
- `llama3.1:70b` - Best quality, slow (40 GB)
- `mistral:7b` - Alternative, fast (4.1 GB)
- `phi3:mini` - Very small, fast (2.3 GB)

### Available VLM Models (via Ollama)
- `llava:7b` - Default (4.7 GB)
- `llava:13b` - Better quality (8 GB)
- `llava:34b` - Best quality, slow (20 GB)
- `bakllava` - LLaVA variant (4.7 GB)
- `llava-phi3` - Smaller, faster (2.9 GB)

### Camera Configuration
```yaml
camera:
  topic: "/cameras/ee/ee_camera/color/image_raw"
  use_compressed: true                    # Use compressed images over network
  depth_topic: "/cameras/ee/ee_camera/aligned_depth_to_color/image_raw"
  camera_info_topic: "/cameras/ee/ee_camera/color/camera_info"
```

### Debug Options
```yaml
debug:
  vlm_debug: false      # Enable verbose VLM logging
  llm_debug: false      # Enable verbose LLM logging
  save_images: true     # Save annotated images to debug_images/ folder
  log_level: "INFO"     # DEBUG, INFO, WARN, ERROR
```

**Image Debugging:**
When `save_images: true`, the VLM saves annotated images to `debug_images/` showing:
- Green crosshair marking the detected object center
- Object name, pixel coordinates, depth, and confidence
- Filename format: `detect_<object>_<timestamp>.jpg`
- Example: `detect_red_cup_20260216_143052.jpg`

This helps debug object localization issues visually!
