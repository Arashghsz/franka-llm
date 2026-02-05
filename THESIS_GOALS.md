https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_stereo.html# 🎯 Thesis Goals: Distributed Edge AI for Robotic Manipulation

**Status**: In Progress (Feb 2026)  
**Target Conference**: Ro-Man 2026  
**Platform**: Franka FR3 + Jetson AGX Orin + ROS 2

---

## High-Level System Architecture

```
┌─────────────────────────────────────────────────────────┐
│              JETSON AGX ORIN (Edge AI)                  │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  LLM Node (Task Planner)                         │  │
│  │  - Model: llama3.1:8b (local)                    │  │
│  │  - Input: /user_command (natural language)       │  │
│  │  - Output: /planned_action (JSON task plan)      │  │
│  └──────────────────────────────────────────────────┘  │
│                      ↓                                  │
│  ┌──────────────────────────────────────────────────┐  │
│  │  VLM Safety Agent                                │  │
│  │  - Input: /camera/image + /planned_action        │  │
│  │  - Output: /vlm_safety_verdict (SAFE/UNSAFE)     │  │
│  │  - Checks: obstacles, hazards, feasibility       │  │
│  └──────────────────────────────────────────────────┘  │
│                      ↓                                  │
│  ┌──────────────────────────────────────────────────┐  │
│  │  RealSense Camera (RGB-D)                        │  │
│  │  - Publishes: /camera/color/image_raw            │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
         ↕ ROS 2 Network (pub/sub)
┌─────────────────────────────────────────────────────────┐
│         CONTROLLER PC (Execution Layer)                 │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Coordinator Node                                │  │
│  │  - Orchestrates: LLM → VLM → Motion execution    │  │
│  │  - Subscribes: /vlm_safety_verdict               │  │
│  │  - Publishes: /user_command (to Jetson)          │  │
│  └──────────────────────────────────────────────────┘  │
│                      ↓                                  │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Motion Execution Layer                          │  │
│  │  - MoveIt 2 + cuMotion                           │  │
│  │  - Franka Robot Control                          │  │
│  │  - Publishes: /execution_status                  │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## Implementation Status

### Phase 0: Perception & 3D Mapping (Jetson) - **COMPLETE** ✅
- [x] RealSense D415 camera plugged into Jetson (fixed, overhead viewpoint)
- [x] Isaac ROS RealSense driver running (publishes depth + RGB)
- [x] Isaac ROS NVBlox 3D reconstruction pipeline active
- [x] 3D scene map published on `/nvblox_node/static_esdf_pointcloud`
- [x] Network verified: RTX6000 receives all topics over ROS_DOMAIN_ID=42

**Status**: Camera feed and 3D reconstruction flowing from Jetson → RTX6000 over ROS 2 network.

**Location**: Jetson workspace (`~/workspaces/isaac_ros-dev/`), launched via:
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py run_realsense:=True mode:=static
```

### Phase 1: Task Planner (LLM) - **IN PROGRESS**
- [x] Basic LLM node with Ollama integration
- [x] ROS 2 pub/sub setup (subscribes `/user_command`, publishes `/llm_response`)
- [x] System prompt for task planning
- [ ] Enhanced: Context tracking (conversation history)
- [ ] Enhanced: Error handling for safety rejections
- [ ] Enhanced: State machine (IDLE → PLANNING → AWAITING_SAFETY → READY)
- [ ] Enhanced: Timeout handling for VLM verdicts
- [ ] Conversation logging node (logs `/user_command` and `/llm_response` to JSONL)
- [ ] CLI chat node (terminal interface for interacting with LLM)

**Location**: `/home/arash/franka-llm/src/franka_llm_planner/franka_llm_planner/llm_node.py`

**Next**: Iteration 1 – Add conversation logging + CLI chat interface.

### Phase 2: Safety Agent (VLM) - **SCAFFOLDED** (stub ready)
- [x] VLM package structure created
- [x] Stub node that subscribes `/planned_action` and `/camera/color/image_raw`
- [ ] Define final `/vlm_request` and `/vlm_safety_verdict` topics
- [ ] Integrate real VLM model (LLaVA / VILA from Isaac ROS index)
- [ ] Safety check implementation (obstacle, hazard, grasp feasibility)
- [ ] ROS 2 integration for network communication

**Location**: `/home/arash/franka-llm/src/franka_vlm_safety/franka_vlm_safety/vlm_node.py`

**Status**: Waiting for Iteration 2 of multi-agent architecture.

### Phase 3: Motion Execution - **PARTIAL**
- [x] Basic joint control (demo.py on RTX6000)
- [x] MoveIt 2 integration
- [x] Consolidated codebase structure:
  - `moveit/robot.py` - FrankaHelperReal (core robot interface)
  - `moveit/manipulation_tasks.py` - FrankaManipulation (high-level tasks)
  - `moveit/moveit_helpers.py` - MoveIt2 utilities
- [ ] **NEW: Perception-aware planning** – Subscribe to `/nvblox_node/static_esdf_pointcloud` and add dynamic collision objects to MoveIt scene
- [ ] Pick & place primitives
- [ ] Integration with safety verdicts
- [ ] Feedback loop (`/execution_status`) to coordinator

**Location**: `/home/arash/franka-llm/src/franka_llm_planner/franka_llm_planner/moveit/` (Jetson) and RTX6000 motion executor

**Next**: Build perception→MoveIt bridge on RTX6000 to make motion planning "camera-aware".

### Phase 4: Coordinator - **NOT STARTED**
- [ ] LLM → VLM → Motion orchestration
- [ ] Subscribes: `/planned_action`, `/vlm_safety_verdict`
- [ ] Publishes: `/user_command` (to Jetson), `/execution_status` (from motion)
- [ ] State management
- [ ] Error recovery and timeout handling

---

## Robot Action Policies (Primitives)

### Defined Actions (might need some change)
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
        "note": "Uses vision to verify object properties (Osama's recent work)"
    }
}
```

### Potential Future Actions
- `push()` - manipulate objects without grasping
- `rotate()` - reorient grasped object
- `handover()` - pass object to human
- `deform()` - handle deformable objects

---

## Thesis Objectives

### Primary Goal
**Distributed Edge AI Architecture for Robotic Pick & Place**

Implement and evaluate a distributed system where:
1. **LLM** (Task Planner) on Jetson reasons about high-level tasks
2. **VLM** (Safety Agent) on Jetson verifies visual safety
3. **Motion Execution** on separate PC handles low-level control
4. All communicate via ROS 2 pub/sub over local network

### Multi-Agent & Conversation Logging Goal

Build a **multi-agent LLM + VLM system** where:
- LLM and VLM run as separate ROS 2 nodes (agents) on Jetson
- Coordinator and Motion Execution run on the RTX6000 controller
- All inter-agent messages (user → LLM → VLM → Coordinator → Motion) are **logged as a chat-like transcript**

Planned iterations:
1. **Iteration 1** – Single-agent LLM with logging
  - Terminal/chat interface for `/user_command` ↔ `/llm_response`
  - Conversation logger node writing JSONL logs of all messages
2. **Iteration 2** – Add VLM agent
  - Define `/vlm_request` and `/vlm_safety_verdict` topics
  - Log LLM↔VLM interactions alongside user messages
3. **Iteration 3** – Connect to Motion Execution
  - Coordinator on RTX6000 subscribes to planned actions and safety verdicts
  - Motion status (`/execution_status`) included in the same conversation log

### Secondary Goals

#### 1. Task Evaluation (Ro-Man 2026 Paper)
- [ ] Implement 3-5 pick & place scenarios
  - [ ] Single object, simple background
  - [ ] Multiple objects, complex background
  - [ ] Occluded objects
  - [ ] Different object types (rigid, deformable, fragile)
  - [ ] Different target locations (shelves, bins, tables)

#### 2. Model Benchmarking
Evaluate and compare:
- **LLMs**: llama3.1:8b, Mistral, local quantized models
- **VLMs**: LLaVA, VILA (NVIDIA), LLaMA-Vision
- **Motion Planning**: MoveIt + cuMotion, trajectory optimization
- **Metrics**: 
  - [ ] Inference latency (ms)
  - [ ] Memory usage (GB)
  - [ ] Success rate (%)
  - [ ] End-to-end execution time (s)
  - [ ] Safety verdict accuracy

#### 3. Distributed System Optimization
- [ ] ROS 2 network performance (DDS tuning)
- [ ] Deterministic execution in disconnected environments
- [ ] Fallback strategies when network is unstable
- [ ] Concurrent LLM/VLM execution efficiency

#### 4. Safety & Robustness
- [ ] VLM safety checks: obstacle detection, collision avoidance
- [ ] Grasp feasibility verification
- [ ] Error recovery mechanisms
- [ ] Human-in-the-loop intervention

#### 5. Multi-Agent Behavior Analysis
- [ ] Design logging format for multi-agent conversations (JSONL)
- [ ] Implement ROS 2 logger node for user, LLM, VLM, and coordinator messages
- [ ] Build simple chatbot-style interface to inspect conversations
- [ ] Analyze failure cases and emergent behaviors from logs

---

## Expected Results & Metrics

### Success Criteria
| Metric | Target | Evaluation Method |
|--------|--------|-------------------|
| **Task Success Rate** | > 85% | # successful pickups / total attempts |
| **LLM Inference Time** | < 2s | Measure on Jetson AGX Orin |
| **VLM Inference Time** | < 1s | Visual check + verdict generation |
| **E2E Execution Time** | < 10s | From user command to completion |
| **Safety Accuracy** | > 95% | Manual verification of verdicts |
| **Network Latency** | < 500ms | ROS 2 message propagation |

### Evaluation Datasets
- [ ] Custom pick & place scenarios (20+ trials per condition)
- [ ] Object diversity: 5+ different objects
- [ ] Background complexity: simple, cluttered, textured
- [ ] Lighting conditions: varied

---
## Technology Stack

### Hardware
- **Robot**: Franka Research 3 (FR3)
- **Edge AI**: Jetson AGX Orin (MAXN mode)
- **Sensor**: Intel RealSense D435i (RGB-D)
- **Compute**: Secondary PC for motion control

### Software
- **Middleware**: ROS 2 Humble, Jazzy on PC controler
- **LLM**: llama3.1:8b (via Ollama)
- **VLM**: LLaVA / VILA (to be selected)
- **Motion Planning**: MoveIt 2
- **Vision**: OpenCV, MediaPipe
- **Language**: Python 3.10+

### Optimization Targets
- Jetson AGX Orin local inference (no cloud APIs)
- Quantized models (INT8, FP16) for speed
- ROS 2 with domain ID isolation
- GPU memory optimization

---

## References & Related Work

### Key Papers
- **SayCan** - LLM + Affordances: https://say-can.github.io/
- **Code as Policies** - LLM → Code → Robot: https://code-as-policies.github.io/
- **RT-2** - Vision-Language-Action models: https://robotics-transformer2.github.io/
- **VILA** - NVIDIA's efficient VLM

### Platforms
- **Isaac ROS**: https://nvidia-isaac-ros.github.io/
- **MoveIt 2**: https://moveit.picknik.ai/
- **ROS 2**: https://docs.ros.org/en/humble/

---

## Next Steps (Immediate)

### **Iteration 1 (Jetson): Multi-Agent Logging & Chat**
Goal: See LLM and VLM agents "talking" with conversation logs.

- [ ] Implement `conversation_logger` node – subscribes to `/user_command`, `/llm_response`, later `/vlm_request`, `/vlm_safety_verdict` and writes JSONL logs with timestamps
- [ ] Implement `cli_chat_node` – terminal interface to send commands to LLM and receive responses
- [ ] Test locally: type a command → LLM responds → see full conversation in logs

**Timeline**: This week  
**Output**: `logs/agent_chat.jsonl` file + terminal chat experience

---

### **Phase 3.5 (RTX6000): Perception-Aware Planning**
Goal: Make MoveIt aware of obstacles from the 3D camera map.

- [ ] Build a `perception_to_moveit_bridge` node on RTX6000 that:
  - Subscribes to `/nvblox_node/static_esdf_pointcloud` (3D map from Jetson)
  - Converts point cloud to collision objects (e.g., bounding boxes)
  - Adds/updates obstacles in MoveIt's planning scene via `PlanningSceneInterface`
- [ ] Test: Place visible objects in front of robot → verify they appear as obstacles in RViz → simple motion plan should route around them
- [ ] Expose via a ROS 2 service or auto-update when new 3D data arrives

**Timeline**: Next week  
**Outcome**: MoveIt plans collision-free paths considering real camera data

---

### **Iteration 2 (Jetson): Add VLM Agent**
Goal: LLM → VLM safety check → verdict.

- [ ] Define `/vlm_request` (input: planned_action JSON + image reference)
- [ ] Define `/vlm_safety_verdict` (output: SAFE/UNSAFE + reason)
- [ ] Replace stub VLM with real model (LLaVA or VILA from Isaac ROS index)
- [ ] Extend logger to capture VLM messages
- [ ] Test: LLM produces plan → VLM checks → verdict logged

**Timeline**: Following week  
**Output**: Full `user → LLM → VLM → verdict` conversation log

---

### **Iteration 3 (RTX6000): Connect Coordinator + Motion**
Goal: Orchestrate the full LLM → VLM → Motion pipeline.

- [ ] Build `coordinator_node` on RTX6000:
  - Subscribes to `/planned_action` and `/vlm_safety_verdict`
  - If SAFE, calls motion executor (your existing demo.py refactored as a service)
  - Publishes `/execution_status` (RUNNING, SUCCESS, FAILURE)
- [ ] Extend logger to include `/execution_status`
- [ ] Test end-to-end: user command → full pipeline → robot moves → logs show everything

**Timeline**: End of month  
**Outcome**: Full multi-agent system with complete audit trail

---

### **Beyond**: Real-World Validation
- [ ] Run 3–5 pick & place scenarios in simulation
- [ ] Benchmark: latency, success rate, safety accuracy
- [ ] Deploy on real Franka FR3 hardware

## Notes

- **Network Setup**: ROS_DOMAIN_ID=42 for local testing (verified working Feb 5, 2026)
- **Ollama Setup**: Running on Jetson, accessible via http://localhost:11434
- **Camera Setup**: Intel RealSense D415 plugged into Jetson (fixed, overhead mounting)
- **3D Mapping**: Isaac ROS NVBlox outputs 3D scene reconstruction at ~10 Hz on `/nvblox_node/static_esdf_pointcloud`
- **Development Environment**: VS Code with ROS 2 extension
- **Backup Strategy**: All code versioned in Git
- **Isaac ROS Packages Used**:
  - `isaac_ros_realsense` – depth camera driver
  - `isaac_ros_nvblox` – 3D scene reconstruction
  - `nvblox_examples_bringup` – pre-configured launch files
  - `nvblox_ros`, `nvblox_msgs`, `nvblox_rviz_plugin` – utilities

---

**Last Updated**: Feb 5, 2026  
**Authored By**: Arash  
**Status**: Active Development – Phase 0 complete, Iteration 1 ready to start
