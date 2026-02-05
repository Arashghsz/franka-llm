# 🎯 Thesis Goals: Distributed Edge AI for Robotic Manipulation

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

### Phase 1: Task Planner (LLM) - **IN PROGRESS**
- [x] Basic LLM node with Ollama integration
- [x] ROS 2 pub/sub setup
- [x] System prompt for task planning
- [ ] Enhanced: Context tracking (conversation history)
- [ ] Enhanced: Error handling for safety rejections
- [ ] Enhanced: State machine (IDLE → PLANNING → AWAITING_SAFETY → READY)
- [ ] Enhanced: Timeout handling for VLM verdicts

**Location**: `/home/arash/franka-llm/src/franka_llm_planner/franka_llm_planner/llm_node.py`

### Phase 2: Safety Agent (VLM) - **NOT STARTED**
- [ ] VLM node creation (camera feed processor)
- [ ] Model selection (LLaVA / VILA / GPT-4V)
- [ ] Safety check implementation
- [ ] ROS 2 integration for network communication

**Location**: `/home/arash/franka-llm/src/franka_vlm_safety/` (to be created)

### Phase 3: Motion Execution - **PARTIAL**
- [x] Basic joint control (demo.py)
- [x] MoveIt integration
- [x] Consolidated codebase structure:
  - `moveit/robot.py` - FrankaHelperReal (core robot interface)
  - `moveit/manipulation_tasks.py` - FrankaManipulation (high-level tasks)
  - `moveit/moveit_helpers.py` - MoveIt2 utilities
- [ ] Pick & place primitives
- [ ] Integration with safety verdicts
- [ ] Feedback loop to coordinator

**Location**: `/home/arash/franka-llm/src/franka_llm_planner/franka_llm_planner/moveit/`
  - All motion control in single folder for easy maintenance
  - Imported via main package `__init__.py`

### Phase 4: Coordinator - **NOT STARTED**
- [ ] LLM → VLM → Motion orchestration
- [ ] State management
- [ ] Error recovery

---

## Robot Action Policies (Primitives)

### Defined Actions
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

1. **Enhance LLM Node** (this week)
   - Add state machine
   - Add conversation history
   - Add timeout handling

2. **Create VLM Safety Node** (next week)
   - Subscribe to camera feed
   - Subscribe to planned action
   - Publish safety verdict

3. **Implement Coordinator** (following week)
   - Orchestrate LLM → VLM → Motion cycle
   - Handle state transitions
   - Log all interactions

4. **End-to-end Testing**
   - Gazebo simulation first
   - Then real hardware

---

## Notes

- **Network Setup**: ROS_DOMAIN_ID=42 for local testing
- **Ollama Setup**: Running on Jetson, accessible via http://localhost:11434
- **Development Environment**: VS Code with ROS 2 extension
- **Backup Strategy**: All code versioned in Git

---

**Last Updated**: Feb 4, 2026  
**Authored By**: Arash  
**Status**: Active Development
