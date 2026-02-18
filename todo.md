# Goals: Distributed Edge AI for Robotic Manipulation

**Status**: In Progress (Feb 2026)  
**Platform**: Franka FR3 + Jetson AGX Orin + ROS 2

---

## High-Level System Architecture (LLM → VLM → Detection → Motion)

```
┌──────────────────────────────────────────────────────────────┐
│                   JETSON AGX ORIN (Edge AI)                  │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  LLM Node (Task Planner)                               │  │
│  │  - Input: /user_command                                │  │
│  │  - Output: /planned_action (JSON plan)                 │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  VLM Agent (Vision + Detection)                        │  │
│  │  - Input: /vlm_request (image + plan)                  │  │
│  │  - Output: /target_detection (object, pixel, depth)    │  │
│  │  - Provides: label, bbox, pixel coords, depth          │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
         ↕ ROS 2 Network (pub/sub)
┌──────────────────────────────────────────────────────────────┐
│                 CONTROLLER PC (Execution)                    │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  RealSense Camera (RGB-D)                              │  │
│  │  - Streams images to Jetson for VLM processing         │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Coordinate Transformation Node                        │  │
│  │  - Input: /target_detection (pixel + depth from VLM)   │  │
│  │  - Output: /target_pose (x, y, z in robot frame)       │  │
│  │  - Converts pixel+depth → robot coordinates            │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Coordinator Node                                      │  │
│  │  - Orchestrates: LLM → VLM → Transform → Motion        │  │
│  │  - Handles user confirmation before execution          │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Motion Execution Layer                                │  │
│  │  - MoveIt 2 + Franka Control                           │  │
│  │  - Publishes: /execution_status                        │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘

UI: Single-page web dashboard (HTML/CSS/JS + jQuery) shows chat, live camera, status, and confirmation.
```

---

## Deployment Split

- **Jetson**: LLM + VLM (with integrated object detection)
- **Controller PC**: Cameras + Coordinate Transform + Coordinator + Motion Execution + UI + Conversation Logger

---

## Implementation Status


### Phase 1: Task Planner (LLM) - **IN PROGRESS**
- [x] Basic LLM node with Ollama integration
- [x] ROS 2 pub/sub setup (subscribes `/user_command`, publishes `/llm_response`)
- [x] System prompt for task planning
- [ ] Context tracking (conversation history)
- [ ] State machine (IDLE → PLANNING → AWAITING_VLM → READY)
- [ ] Timeout handling for VLM/Detection
- [ ] Conversation logging (JSONL)
- [ ] CLI chat node

### Phase 2: VLM Agent (Vision + Detection) - **IN PROGRESS**
- [x] VLM package structure created
- [x] Object detection working (outputs object label, pixel coords, depth)
- [x] Current detection: "red dice" at pixel (870, 270), depth 0.478m
- [ ] Define `/vlm_request` topic (image + task from LLM)
- [ ] Define `/target_detection` topic (object, pixel, depth)
- [ ] Integrate with LLM task planning
- [ ] Handle multiple objects in scene

### Phase 3: Coordinate Transformation - **NOT STARTED** 🔴 (Controller PC)
- [ ] Create transformation node
- [ ] Subscribe to `/target_detection` (pixel + depth from VLM)
- [ ] Subscribe to `/tf` (camera → robot base transform)
- [ ] Subscribe to `/camera_info` (camera intrinsics)
- [ ] Publish `/target_pose` (x, y, z in robot base frame)
- [ ] Test with known object positions
- [ ] Add RViz visualization of detected 3D points

### Phase 4: Motion Execution - **WORKING** ✅ (Controller PC)
- [x] Basic joint control (demo.py on RTX6000)
- [x] MoveIt 2 integration
- [x] Cartesian position control (x, y, z relative to base)
- [x] Gripper control (open/close)
- [x] Fixed gripper orientation for grasping (rx=0, ry=π, rz=2.45)
- [x] Velocity scaling control
- [ ] Pick & place primitives (grasp sequence)
- [ ] Camera-to-robot coordinate transformation
- [ ] Perception-aware planning (use target detection + 3D map)
- [ ] Feedback loop (`/execution_status`) to coordinator

### Phase 5: Coordinator - **NOT STARTED** (Controller PC)
- [ ] LLM → VLM → YOLO → Motion orchestration
- [ ] Subscribes: `/planned_action`, `/vlm_grounding`, `/target_detection`
- [ ] Publishes: `/execution_status`, `/execution_goal`
- [ ] Requires user confirmation before execution
- [ ] Error recovery and timeout handling

### Phase 6: UI (Single-Page Dashboard) - **NOT STARTED** (Controller PC)
- [ ] Chat panel (all agents)
- [x] User input box
- [x] Live camera feed
- [x] Status panel
- [ ] Confirmation step before execution
- [x] Chat starts with assistant greeting (e.g., "Hi, I'm Franka, your assistant")
- [ ] Every agent/status log appears in the chat timeline

---

## Robot Action Policies (Primitives)

```python
ROBOT_ACTIONS = {
    "pick": {
        "parameters": ["object", "location", "grasp_type"],
        "preconditions": ["object_detected", "path_clear"],
        "postconditions": ["object_grasped"],
    },
    "place": {
        "parameters": ["object", "target_location", "orientation"],
        "preconditions": ["object_grasped", "target_clear"],
        "postconditions": ["object_placed"],
    },
    "move": {
        "parameters": ["target_pose", "velocity"],
        "preconditions": ["path_clear"],
        "postconditions": ["at_target_pose"],
    },
    "wait": {
        "parameters": ["duration"],
        "preconditions": [],
        "postconditions": ["waited"],
    },
    "inspect": {
        "parameters": ["object", "aspect"],
        "preconditions": [],
        "postconditions": ["inspection_complete"],
        "note": "Uses vision to verify object properties"
    }
}
```

---

## Thesis Objectives (Updated)

### Primary Goal
**Distributed Edge AI Architecture for Robotic Pick & Place**

Implement and evaluate a distributed system where:
1. **LLM** (Task Planner) generates high-level intent
2. **VLM** (Semantic Grounding) identifies the intended object
3. **YOLO** (Precise Localization) provides bbox + depth
4. **Motion Execution** handles low-level control
5. **Coordinator** enforces safety, confirmation, and orchestration

### Multi-Agent & Conversation Logging Goal
- All inter-agent messages (user → LLM → VLM → YOLO → Coordinator → Motion) are logged as a chat-like transcript.
- UI shows full agent chat + execution status + confirmation step.

---

## 🎯 NEXT PRIORITY TASKS (Feb 2026)

### **IMMEDIATE (This Week)**

#### 1. **Camera-to-Robot Coordinate Transformation** 🔴 CRITICAL
**Goal**: Convert pixel (870, 270) + depth (0.478m) → robot coordinates (x, y, z)

**Implementation**:
- Create transformation node in `franka_vision_detection` package
- Subscribe to:
  - `/cameras/ee/ee_camera/aligned_depth_to_color/image_raw` (depth)
  - `/cameras/ee/ee_camera/color/camera_info` (intrinsics)
  - `/detection/detections` (YOLO output)
  - `/tf` (camera → robot base transform)
- Publish to:
  - `/target_pose` (geometry_msgs/PoseStamped with x, y, z in robot base frame)

**Algorithm**:
```python
# 1. Get pixel (u, v) from YOLO bbox center
# 2. Get depth d from aligned depth image at (u, v)
# 3. Convert to 3D camera coordinates using camera intrinsics:
#    X_cam = (u - cx) * d / fx
#    Y_cam = (v - cy) * d / fy
#    Z_cam = d
# 4. Transform from camera frame to robot base frame using TF:
#    P_base = T_base_camera * P_camera
```

**Files to create**:
- `src/franka_vision_detection/franka_vision_detection/coordinate_transform.py`

**Testing**:
- Place red dice at known position
- Verify transformed coordinates match expected robot coordinates
- Add visualization in RViz showing detected 3D point

---

#### 2. **Pick-and-Place Primitive** 🟡 HIGH PRIORITY
**Goal**: Create automated pick-and-place sequence using detected coordinates

**Implementation** (in `franka_motion_executor`):
```python
def pick_at_position(x, y, z, approach_height=0.3, grasp_width=0.03):
    """
    Pick object at detected position
    1. Move to approach position (x, y, approach_height)
    2. Open gripper to grasp_width
    3. Move down to grasp height (x, y, z)
    4. Close gripper
    5. Lift to approach height
    """
    
def place_at_position(x, y, z, approach_height=0.3):
    """
    Place object at target position
    1. Move to approach position
    2. Move down to place height
    3. Open gripper
    4. Lift away
    """
```

**Testing**:
- Manual test: `pick_at_position(0.5, 0.0, 0.15)`
- Verify gripper grasps successfully
- Test with different object heights

---

#### 3. **End-to-End Detection → Grasp Pipeline** 🟢 INTEGRATION
**Goal**: Connect YOLO detection → coordinate transform → motion execution

**Implementation**:
Create simple integration node:
```python
# src/franka_coordinator/franka_coordinator/simple_pick.py

class SimplePickNode:
    def __init__(self):
        self.target_sub = self.create_subscription(
            PoseStamped, '/target_pose', self.target_callback, 10)
        self.manip = FrankaManipulation(helper)
    
    def target_callback(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.logger.info(f"Picking object at ({x:.3f}, {y:.3f}, {z:.3f})")
        self.manip.pick_at_position(x, y, z)
```

**Testing**:
- Run full pipeline: detection → transform → pick
- Place dice, detect it, robot picks it up
- Verify success rate

---

### **SHORT TERM (Next 2 Weeks)**

#### 4. **VLM Integration for Semantic Grounding**
- Deploy LLaVA/VILA on Jetson
- Create `/vlm_request` and `/vlm_grounding` topics
- Test: "Pick the red dice" → VLM identifies "red dice" → YOLO filters for it

#### 5. **Safety & Confirmation Layer**
- Add user confirmation before executing motion
- Implement workspace boundaries check
- Add collision detection

#### 6. **UI Dashboard**
- Live camera feed with detection overlay
- Agent conversation log
- Confirm/Cancel buttons
- Execution status display

---

### **MEDIUM TERM (Month 2-3)**

#### 7. **LLM Task Planning**
- Context tracking for multi-step tasks
- State machine implementation
- Conversation logging to JSONL

#### 8. **Performance Evaluation**
- Latency measurements (detection → execution)
- Success rate tracking
- Edge AI vs Cloud comparison

---

## 📋 Current Status Summary

**✅ WORKING**:
- YOLO detection (pixel + depth)
- Motion control (x, y, z commands)
- Gripper control
- MoveIt2 integration

**🔴 BLOCKING**:
- Camera-to-robot coordinate transformation
- Pick-and-place primitives

**⏳ NEXT UP**:
- End-to-end detection → grasp pipeline
- VLM semantic grounding
- UI starts with an assistant greeting and keeps a full timeline of logs.

### Task Evaluation
- [ ] Implement 3-5 pick & place scenarios
- [ ] Compare LLM-only, VLM-only, YOLO-only, and Hybrid (LLM+VLM+YOLO)
- [ ] Metrics: success rate, latency, safety accuracy, human confirmation rate

---
