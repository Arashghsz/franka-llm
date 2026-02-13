# Quick Start: VLM-Only Pick & Place Pipeline

## Architecture Flow

```
User Command → LLM (parse) → VLM (bbox) → Depth Resolver (3D) → Motion
```

## Terminal Setup (4 terminals)

### Terminal 1: RealSense Cameras
```bash
cd ~/franka-multiagent-manipulation
source install/setup.zsh
ros2 launch realsense_cameras cameras.launch.py
```

### Terminal 2: LLM Task Planner
```bash
cd ~/franka-multiagent-manipulation
source install/setup.zsh
ros2 run franka_llm_planner llm_node
```

### Terminal 3: VLM Grounding Agent
```bash
cd ~/franka-multiagent-manipulation
source install/setup.zsh
ros2 run franka_vlm_agent vlm_node
# Note: Auto-analyze is disabled by default - VLM only processes when requested
# Uses EE camera compressed images by default for network efficiency
```

### Terminal 4: Coordinator (Depth Resolver)
```bash
cd ~/franka-multiagent-manipulation
source install/setup.zsh
ros2 run franka_coordinator coordinator_node
```

### Terminal 5: Interactive Chat
```bash
cd ~/franka-multiagent-manipulation
source install/setup.zsh
python3 src/simple_terminal_chat.py
```

## Example Usage

### Conversational Inspection
```
> what do you see at the table?
[LLM] Action: inspect, Target: table
[VLM] I can see several objects on the table including a red plastic cup 
      on the left side, a green apple in the center, and a screwdriver 
      near the right edge. The table surface appears to be light colored.
```

### Object Localization (Pick & Place)
```
> pick up the red cup
[LLM] Action: pick, Target: red cup
[VLM] Bbox: [234, 156, 310, 245]
[Depth Resolver] 3D Position: X: 0.523m, Y: 0.142m, Z: 0.650m
```

## Topics Reference

| Topic | Type | Purpose |
|-------|------|---------|
| `/user_command` | String | User input to LLM |
| `/llm_response` | String | LLM parsed action plan (JSON) |
| `/vlm_request` | String | Request to VLM with target object |
| `/vlm_grounding` | String | VLM bbox result (JSON) |
| `/target_position` | PoseStamped | 3D coordinates in camera frame |

## Testing Without LLM

If you want to skip LLM and directly test VLM grounding:

```bash
# Terminal 1-3: Same as above (cameras, VLM, coordinator)

# Terminal 4: Direct VLM test
python3 src/test_vlm_grounding.py
# Enter: "red cup"
```

## Current State

✅ VLM node accepts grounding requests (on-demand only)
✅ VLM prompts for bounding boxes (LLaVA text-based)
✅ VLM uses EE camera compressed images by default
✅ Coordinator resolves depth from bbox
✅ 3D position published to `/target_position`
✅ Chatbot mode for scene inspection
⚠️ Motion execution not yet connected
⚠️ TF camera→robot transform needed

## Next Steps for Full Pipeline

1. **Better VLM models** - Florence-2, Qwen-VL have native bbox output
2. **TF transforms** - Convert camera coords to robot base frame
3. **Motion integration** - Send 3D position to MoveIt planner
4. **User confirmation** - Add safety check before execution
