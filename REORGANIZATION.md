# Architecture Reorganization - February 2026

## What Changed

### Previous Architecture
```
User → LLM Parser → VLM Grounding (bbox) → Coordinator (depth resolver) → Motion
```

**Issues:**
- Bounding box grounding was complex and unreliable with LLaVA
- Depth resolution was separated from VLM
- LLM just parsed commands, didn't route intelligently
- Coordinator was just a pass-through with depth logic

### New Architecture
```
User → LLM Coordinator (conversational routing) → VLM (object center + depth) OR Motion
```

**Benefits:**
- **Object-Centric**: VLM finds the **center of target object**, not just image center
- **Conversational**: Coordinator greets users, introduces itself, handles small talk
- **Smarter**: LLM decides which agent to use based on intent
- **Cleaner**: Each agent has clear, focused responsibility
- **Integrated**: VLM handles both object localization and depth

## File Changes

### Created Files
- `src/franka_llm_planner/franka_llm_planner/llm_coordinator_node.py` - New intelligent coordinator
- `src/test_depth_center_pixel.py` - Test utility for depth sampling
- `src/chat_coordinator.py` - Updated chat interface for new architecture
- `REORGANIZATION.md` - This file

### Modified Files
- `src/franka_vlm_agent/franka_vlm_agent/vlm_node.py` - Added depth resolution, removed bbox grounding
- `src/franka_llm_planner/setup.py` - Added llm_coordinator entry point
- `QUICKSTART.md` - Updated documentation for new architecture

### Deprecated Files (keep for reference)
- `src/franka_coordinator/franka_coordinator/coordinator_node.py` - Old coordinator (now LLM does routing)
- `src/test_vlm_grounding.py` - Old bbox grounding test
- `src/simple_terminal_chat.py` - Old chat interface

## New Workflow

### 1. Test Depth Sampling First
```bash
# Terminal 1: Start cameras
ros2 launch realsense_cameras cameras.launch.py

# Terminal 2: Test depth sampling
python3 src/test_depth_center_pixel.py
```

You should see continuous output of center pixel depth and 3D position.

### 2. Run VLM with Depth
```bash
# Terminal 1: Cameras (as above)

# Terminal 2: VLM node
ros2 run franka_vlm_agent vlm_node

# Terminal 3: Send test request
ros2 topic pub --once /vlm_request std_msgs/String \
  'data: "{\"type\": \"scene_description\"}"'

# Terminal 4: Listen for results
ros2 topic echo /vlm/explanation
ros2 topic echo /vlm_center_position
```

### 3. Run Full Pipeline with Coordinator
```bash
# Terminal 1: Cameras
ros2 launch realsense_cameras cameras.launch.py

# Terminal 2: LLM Coordinator
ros2 run franka_llm_planner llm_coordinator

# Terminal 3: VLM Agent
ros2 run franka_vlm_agent vlm_node

# Terminal 4: Chat interface
python3 src/chat_coordinator.py
```

Then try commands like:
- "what do you see on the table?"
- "describe the scene"
- "where is the apple?"

## Key Topic Changes

### Old Topics
- `/llm/response` - LLM parsing output
- `/vlm_grounding` - Bounding box results
- `/target_position` - 3D from coordinator

### New Topics
- `/coordinator_response` - Coordinator responses
- `/vlm/explanation` - Scene descriptions
- `/vlm_center_position` - 3D position from VLM (center pixel)
- `/vlm_request` - Requests to VLM

## Agent Responsibilities

### LLM Coordinator
- **Input**: `/user_command` (natural language)
- **Output**: Routes to `/vlm_request` OR `/motion_command` OR direct response
- **Job**: 
  - Handle greetings and small talk ("hi", "who are you", etc.)
  - Understand task intent
  - Decide which agent(s) to use
  - For object manipulation, request object localization from VLM
- **LLM Model**: Llama 3.1:8b (configurable)
- **Conversational**: Introduces itself as "Franka Assistant", responds to greetings

### VLM Agent
- **Input**: `/vlm_request` (scene_description or locate with object name)
- **Output**: 
  - `/vlm/explanation` (natural language scene description or object location)
  - `/vlm_center_position` (3D coords of **object center pixel**)
- **Job**: Analyze scene OR locate specific object and find its center point
- **VLM Model**: LLaVA:7b (configurable)
- **Object Localization**: VLM identifies object and returns its center pixel coordinates
- **Depth**: Samples depth at the object's center pixel
- **Camera Info**: Gets from `/cameras/ee/ee_camera/color/camera_info`

**How it works:**
1. User: "pick up the red cup"
2. Coordinator asks VLM to locate "red cup"
3. VLM analyzes image, finds red cup, returns center pixel (e.g., [180, 240])
4. VLM gets depth at that pixel (e.g., 0.62m)
5. VLM publishes 3D position of cup's center

### Motion Agent (Future)
- **Input**: `/motion_command` (pick, place, move)
- **Output**: `/motion/status` (executing, completed, failed)
- **Job**: Execute physical movements using MoveIt2

## Build and Deploy

```bash
cd ~/franka-llm

# Build new coordinator node
colcon build --packages-select franka_llm_planner

# Source the workspace
source install/setup.zsh

# Test it out
ros2 run franka_llm_planner llm_coordinator --ros-args -p model:=llama3.1:8b
```

## Troubleshooting

### Depth values are 0
- Check that cameras are running: `ros2 topic hz /cameras/ee/ee_camera/aligned_depth_to_color/image_raw`
- Ensure object is within RealSense range (0.2m - 10m)
- Check camera intrinsics are loaded (watch VLM logs)

### VLM not responding
- Verify Ollama is running: `ollama list`
- Check model is loaded: `ollama list | grep llava`
- Test Ollama: `curl http://localhost:11434/api/tags`

### Coordinator not routing
- Check LLM model is loaded: `ollama list | grep llama`
- Verify topic connections: `ros2 topic list | grep vlm_request`
- Check coordinator logs for routing decisions

## Future Enhancements

1. **Multi-point depth sampling** - Sample multiple points instead of just center
2. **Region-based analysis** - VLM analyzes specific regions (left, right, center)
3. **Object tracking** - Track objects across frames using center pixel
4. **Motion integration** - Connect to MoveIt2 for pick and place
5. **TF integration** - Transform camera coords to robot base frame
6. **Better VLMs** - Try Qwen-VL, Florence-2, or VILA for improved scene understanding
