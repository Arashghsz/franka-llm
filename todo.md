# Goals: Distributed Edge AI for Robotic Manipulation

**Status**: In Progress (Feb 2026)  
**Target Conference**: Ro-Man 2026  
**Platform**: Franka FR3 + Jetson AGX Orin + ROS 2

---

## High-Level System Architecture (LLM → VLM → YOLO → Motion)

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
│  │  VLM Agent (Semantic Grounding)                        │  │
│  │  - Input: /vlm_request (image + plan)                  │  │
│  │  - Output: /vlm_grounding (target label + rationale)   │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  YOLO Detector (Precise Localization)                  │  │
│  │  - Input: /vlm_grounding + /camera/color/image_raw     │  │
│  │  - Output: /target_detection (bbox + depth)            │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │  
│  ┌────────────────────────────────────────────────────────┐  │
│  │  RealSense Camera (RGB-D)                              │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
         ↕ ROS 2 Network (pub/sub)
┌──────────────────────────────────────────────────────────────┐
│                 CONTROLLER PC (Execution)                    │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Coordinator Node                                      │  │
│  │  - Orchestrates: LLM → VLM → YOLO → Motion             │  │
│  │  - Handles user confirmation before execution          │  │
│  └────────────────────────────────────────────────────────┘  │
│                       ↓                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  Motion Execution Layer                                │  │
│  │  - MoveIt 2 + cuMotion                                 │  │
│  │  - Publishes: /execution_status                        │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘

UI: Single-page web dashboard (HTML/CSS/JS + jQuery) shows chat, live camera, status, and confirmation.
```

---

## Implementation Status

### Phase 0: Perception & 3D Mapping (Jetson) - **COMPLETE**
- [x] RealSense D415 camera plugged into Jetson (fixed, overhead viewpoint)
- [x] Isaac ROS RealSense driver running (publishes depth + RGB)
- [x] Isaac ROS NVBlox 3D reconstruction pipeline active
- [x] 3D scene map published on `/nvblox_node/static_esdf_pointcloud`
- [x] Network verified: RTX6000 receives all topics over ROS_DOMAIN_ID=42

### Phase 1: Task Planner (LLM) - **IN PROGRESS**
- [x] Basic LLM node with Ollama integration
- [x] ROS 2 pub/sub setup (subscribes `/user_command`, publishes `/llm_response`)
- [x] System prompt for task planning
- [ ] Context tracking (conversation history)
- [ ] State machine (IDLE → PLANNING → AWAITING_GROUNDING → READY)
- [ ] Timeout handling for VLM/YOLO
- [ ] Conversation logging (JSONL)
- [ ] CLI chat node

### Phase 2: VLM Agent (Semantic Grounding) - **SCAFFOLDED**
- [x] VLM package structure created
- [ ] Define `/vlm_request` and `/vlm_grounding` topics
- [ ] Integrate real VLM model (LLaVA / VILA)
- [ ] Output target label and rationale for YOLO filtering

### Phase 3: YOLO Detection (Precise Localization) - **IN PROGRESS**
- [x] YOLO package exists as `franka_vision_detection`
- [ ] Subscribe to `/vlm_grounding` + camera image
- [ ] Publish `/target_detection` (bbox + depth)
- [ ] Validate latency and detection stability

### Phase 4: Motion Execution - **PARTIAL**
- [x] Basic joint control (demo.py on RTX6000)
- [x] MoveIt 2 integration
- [ ] Pick & place primitives
- [ ] Perception-aware planning (use target detection + 3D map)
- [ ] Feedback loop (`/execution_status`) to coordinator

### Phase 5: Coordinator - **NOT STARTED**
- [ ] LLM → VLM → YOLO → Motion orchestration
- [ ] Subscribes: `/planned_action`, `/vlm_grounding`, `/target_detection`
- [ ] Publishes: `/execution_status`, `/execution_goal`
- [ ] Requires user confirmation before execution
- [ ] Error recovery and timeout handling

### Phase 6: UI (Single-Page Dashboard) - **NOT STARTED**
- [ ] Chat panel (all agents)
- [ ] User input box
- [ ] Live camera feed
- [ ] Status panel
- [ ] Confirmation step before execution

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

### Task Evaluation
- [ ] Implement 3-5 pick & place scenarios
- [ ] Compare LLM-only, VLM-only, YOLO-only, and Hybrid (LLM+VLM+YOLO)
- [ ] Metrics: success rate, latency, safety accuracy, human confirmation rate

---

## Next Steps (Immediate)

### Iteration 1: Logging + UI Skeleton
- [ ] Conversation logger (JSONL)
- [ ] Simple SPA UI (HTML/CSS/JS + jQuery)
- [ ] Display chat + camera + status

### Iteration 2: VLM Grounding
- [ ] Define `/vlm_request`, `/vlm_grounding`
- [ ] Connect VLM to camera image

### Iteration 3: YOLO Target Detection
- [ ] Filter YOLO detections using VLM grounding
- [ ] Publish `/target_detection`

### Iteration 4: Full Orchestration
- [ ] Coordinator + confirmation step
- [ ] Motion execution from target detection

---

**Last Updated**: 2026-02-10 15:29:09  
**Authored By**: Arash