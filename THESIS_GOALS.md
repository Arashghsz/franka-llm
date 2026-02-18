# 🎯 Thesis Goals: Distributed Edge AI for Robotic Manipulation

**Status**: In Progress (Feb 2026)  
**Target Conference**: Ro-Man 2026  
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
│  │  VLM Agent (Semantic Grounding + Detection)            │  │
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
│  │  - Input: /target_detection (pixel + depth)            │  │
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
Transform → Motion orchestration
- [ ] Subscribes: `/planned_action`, `/target_detection`, `/target_pose
- [ ] LLM → VLM → YOLO → Motion orchestration
- [ ] Subscribes: `/planned_action`, `/vlm_grounding`, `/target_detection`
- [ ] Publishes: `/execution_status`, `/execution_goal`
- [ ] Requires user confirmation before execution
- [ ] Error recovery and timeout handling

### Phase 6: UI (Single-Page Dashboard) - **NOT STARTED** (Controller PC)
- [ ] Chat panel (all agents)
- [ ] User input box
- [ ] Live camera feed
- [ ] Status panel
- [ ] Confirmation step before execution
- [ ] Chat starts with assistant greeting (e.g., "Hi, I'm Franka, your assistant")
- [ ] Every agent/status log appears in the chat timeline

---

## Thesis Objectives (Updated)

### Primary Goal
**Distributed Edge AI Architecture for Robotic Pick & Place**

Implement and evaluate a distributed system where:
1. **LLM** (Task Planner) generates high-level intent
2. **VLM** (Vision + Detection) identifies and localizes the target object
3. **Coordinate Transform** converts pixel+depth → robot coordinates
4. **Motion Execution** handles low-level control
5. **Coordinator** enforces safety, confirmation, and orchestration

### Multi-Agent & Conversation Logging Goal
- All inter-agent messages (user → LLM → VLM → Coordinator → Motion) are logged as a chat-like transcript.
- UI shows full agent chat + execution status + confirmation step.
- UI starts with an assistant greeting and keeps a full timeline of logs.

### Task Evaluation
- [ ] Implement 3-5 pick & place scenarios
- [ ] Compare LLM-only, VLM-only, and Hybrid (LLM+VLM+Detection)
- [ ] Metrics: success rate, latency, safety accuracy, human confirmation rate

---

**Last Updated**: 2026-02-10 15:44:00  
**Authored By**: Arash