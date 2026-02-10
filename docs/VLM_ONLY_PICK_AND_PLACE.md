# VLM + LLM Pick & Place Architecture Report

**Date**: 2026-02-10 22:21:58  
**Author**: Arash (with Copilot analysis)  
**Status**: Proposal / Architecture Decision  

---

## Summary

This report defines the architecture for performing **pick & place using LLM + VLM only** — no YOLO, no Detectron2, no traditional object detectors. The evaluation focuses on **comparing different VLM and LLM models** against each other in terms of response time, grounding accuracy, localization precision, and pick & place success rate.

Depth is obtained from the existing **RealSense RGB-D camera** by projecting VLM-provided bounding boxes into the depth map.

---

## Architecture: LLM → VLM → Depth → Motion

```
User Command → LLM (plan) → Grounding VLM (identify + locate) → Depth Lookup → Motion (pick & place)
```

- **LLM** generates a task plan from natural language
- **VLM** identifies the target object AND returns pixel-level bounding box
- **Depth** is resolved from RealSense depth map using the VLM's bounding box
- **Motion** executes pick & place via MoveIt 2

**No YOLO. No Detectron2. No task-specific detector at any stage.**

---

## System Architecture

```
┌──────────────────── JETSON AGX ORIN ────────────────────┐
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │  LLM Node (Task Planner)                           │  │
│  │  - Input: /user_command                            │  │
│  │  - Output: /planned_action                         │  │
│  │    {action: "pick", target: "red cup"}             │  │
│  └────────────────────────────────────────────────────┘  │
│                       ↓                                  │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Grounding VLM Node (Semantic + Spatial)           │  │
│  │  - Input: /planned_action + camera image           │  │
│  │  - Output: /vlm_grounding                          │  │
│  │    {target: "red cup",                             │  │
│  │     bbox: [234, 156, 310, 245],                    │  │
│  │     rationale: "red cup on left side"}            │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
         ↕ ROS 2 Network (pub/sub)
┌──────────────────── CONTROLLER PC ──────────────────────┐
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Coordinator + Depth Resolver                      │  │
│  │  - bbox centroid (u,v) → depth_image[v,u] → Z     │  │
│  │  - Deproject (u, v, Z) → 3D point (x, y, z)       │  │
│  │  - Transform to robot frame                        │  │
│  │  - User confirmation                               │  │
│  └────────────────────────────────────────────────────┘  │
│                       ↓                                  │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Motion Executor                                   │  │
│  │  - MoveIt 2 + cuMotion                             │  │
│  │  - Pick & place primitives                         │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

---

## Depth Resolution: VLM Bbox → 3D Coordinates

The key insight: once the VLM provides a **bounding box in pixel coordinates**, we use the RealSense depth image to get the 3D position. No object detector needed.

```python
def resolve_3d_position(bbox, depth_image, camera_intrinsics):
    """Convert VLM bounding box to 3D coordinates"""
    # 1. Get bbox centroid
    cx = (bbox[0] + bbox[2]) / 2
    cy = (bbox[1] + bbox[3]) / 2
    
    # 2. Look up depth at centroid (with median filter for robustness)
    roi = depth_image[bbox[1]:bbox[3], bbox[0]:bbox[2]]
    depth_z = np.median(roi[roi > 0])  # ignore zero (invalid) depths
    
    # 3. Deproject pixel to 3D point using camera intrinsics
    fx, fy = camera_intrinsics.fx, camera_intrinsics.fy
    ppx, ppy = camera_intrinsics.ppx, camera_intrinsics.ppy
    
    x = (cx - ppx) * depth_z / fx
    y = (cy - ppy) * depth_z / fy
    z = depth_z
    
    # 4. Transform from camera frame to robot base frame
    point_robot = tf_camera_to_robot @ [x, y, z, 1]
    
    return point_robot[:3]
```

---

## Evaluation: VLM Model Comparison

The thesis evaluation compares **different VLM and LLM models** against each other — NOT against YOLO/Detectron2. The goal is to find the best VLM+LLM combination for edge-based robotic pick & place.

### VLM Models to Compare

| Model | Parameters | Runtime (Jetson) | Grounding Type | Bbox Output | Notes |
|-------|-----------|:---------------:|----------------|:-----------:|-------|
| **LLaVA 7B** | 7B | ✅ Via Ollama | Text-based (prompt hack for coords) | Approximate | Already running; baseline VLM |
| **LLaVA 13B** | 13B | ⚠️ Quantized | Text-based | Approximate | Heavier, potentially more accurate descriptions |
| **Florence-2** | ~2B | ✅ Native | Native grounding (`CAPTION_TO_PHRASE_GROUNDING`) | Precise bbox | Lightweight, fast, purpose-built for grounding |
| **Qwen-VL / Qwen2-VL** | 7B | ✅ Via Ollama | Native grounding (`<box>` tags) | Precise bbox | Ollama-compatible, structured bbox output |
| **CogVLM** | 17B | ⚠️ Needs quantization | Native grounding | Precise bbox | High accuracy, resource heavy |
| **VILA** | 7B | ✅ Via NVIDIA | Native (NVIDIA optimized for Jetson) | Precise bbox | NVIDIA's VLM, optimized for Jetson hardware |

### LLM Models to Compare (Task Planning)

| Model | Parameters | Runtime (Jetson) | Planning Quality | Notes |
|-------|-----------|:---------------:|:----------------:|-------|
| **Llama 3 8B** | 8B | ✅ Via Ollama | Good | General purpose, strong reasoning |
| **Mistral 7B** | 7B | ✅ Via Ollama | Good | Fast, efficient |
| **Phi-3 Mini** | 3.8B | ✅ Via Ollama | Moderate | Very lightweight, fastest response |
| **Gemma 2 9B** | 9B | ⚠️ Quantized | Good | Google's model, strong instruction following |

### Evaluation Metrics

#### Per-Model Metrics
| Metric | Description | How to Measure |
|--------|-------------|----------------|
| **Inference latency** | Time from image input to VLM response | Timestamp diff (ms) |
| **Grounding accuracy** | Does the VLM identify the correct object? | Manual verification per trial |
| **Bbox precision** | How close is the VLM bbox to the actual object? | IoU against manually annotated ground truth |
| **Depth accuracy** | How close is the resolved 3D point to the real object? | Compare with measured ground truth position (mm) |
| **Pick & place success rate** | Did the robot successfully pick and place? | Binary success/fail per trial |
| **End-to-end latency** | Total time from user command to motion completion | Full pipeline timestamp (s) |
| **Memory usage** | GPU/RAM consumption on Jetson | `tegrastats` monitoring |
| **Open-vocabulary handling** | Can it ground novel/unseen objects? | Test with unusual object descriptions |

#### Comparison Dimensions
| Comparison | What It Reveals |
|-----------|-----------------|
| **VLM A vs VLM B** (same LLM) | Which VLM is best for grounding on Jetson? |
| **LLM A vs LLM B** (same VLM) | Which LLM produces better task plans? |
| **Small vs Large models** | Latency-accuracy tradeoff on edge hardware |
| **Grounding VLM vs Text-only VLM** | Is native bbox output necessary, or can prompt-based coords work? |
| **Quantized vs Full precision** | Impact of quantization on grounding accuracy |

### Evaluation Scenarios (3-5 pick & place tasks)

| Scenario | Command | Difficulty | Tests |
|----------|---------|:----------:|-------|
| **S1: Single object** | "Pick up the red cup" | Easy | Basic grounding + localization |
| **S2: Disambiguation** | "Pick up the cup closest to the robot" | Medium | Spatial reasoning |
| **S3: Multi-step** | "Move the apple to the left side of the table" | Medium | Pick + place + spatial understanding |
| **S4: Open vocabulary** | "Pick up the shiny thing" | Hard | Abstract/novel object description |
| **S5: Cluttered scene** | "Pick up the screwdriver from the toolbox" | Hard | Grounding in complex scenes |

### Results Table Template

| Model Combination | S1 Success | S2 Success | S3 Success | S4 Success | S5 Success | Avg Latency | Bbox IoU | Depth Error |
|-------------------|:----------:|:----------:|:----------:|:----------:|:----------:|:-----------:|:--------:|:-----------:|
| Llama3 + Florence-2 | | | | | | ms | | mm |
| Llama3 + Qwen-VL | | | | | | ms | | mm |
| Llama3 + LLaVA 7B | | | | | | ms | | mm |
| Llama3 + VILA | | | | | | ms | | mm |
| Mistral + Florence-2 | | | | | | ms | | mm |
| Phi-3 + Florence-2 | | | | | | ms | | mm |

---

## Codebase Changes

### Modify: `franka_vlm_agent`
- Make `vlm_node.py` **model-agnostic**: support swapping between VLM backends (LLaVA, Florence-2, Qwen-VL, VILA)
- Add a `vlm_model` parameter to select the active model at launch
- Output structured JSON (bbox + label + rationale) on `/vlm_grounding`

### Modify: `franka_llm_planner`
- Make LLM model **swappable** via parameter (Llama3, Mistral, Phi-3)
- Ensure consistent JSON output format regardless of LLM backend

### Modify: `franka_coordinator`
- Add **depth resolution** logic (bbox → depth → 3D point)
- Subscribe to `/vlm_grounding` with bbox data
- Orchestrate: LLM → VLM → Depth → Motion

### Remove: `franka_vision_detection`
- YOLO/Detectron2 package is **not part of the architecture**
- Can be kept in repo for reference but is not in the pipeline

### Keep As-Is:
- `franka_motion_executor` — executes MoveIt 2 goals
- `realsense_cameras` — publishes RGB-D

---

## Thesis Contribution Statement

> *"We present a distributed LLM+VLM architecture for robotic pick & place running entirely on edge hardware (Jetson AGX Orin), eliminating the need for task-specific object detectors. We evaluate multiple Vision-Language Models (Florence-2, Qwen-VL, LLaVA, VILA) and Large Language Models (Llama 3, Mistral, Phi-3) for their effectiveness in spatial grounding and task planning on a Franka FR3 manipulator, analyzing inference latency, grounding accuracy, and pick & place success rate to determine the optimal model combination for edge-based robotic manipulation."*

---

## Next Steps

1. **Benchmark VLM models on Jetson**: Run Florence-2, Qwen-VL, LLaVA 7B, VILA — measure latency and grounding accuracy
2. **Benchmark LLM models on Jetson**: Run Llama3, Mistral, Phi-3 — measure planning quality and latency
3. **Make `vlm_node.py` model-agnostic**: Support swapping VLM backends via ROS parameter
4. **Implement depth resolution**: bbox → RealSense depth → 3D coordinates
5. **Run evaluation scenarios**: S1-S5 across all model combinations
6. **Analyze results**: Find optimal model combination for edge pick & place

---

**Conclusion**: The thesis focuses purely on **LLM + VLM driven robotic manipulation** — comparing different foundation models for task planning and visual grounding on edge hardware. No traditional object detectors are used. The evaluation determines which VLM and LLM combination provides the best tradeoff between accuracy, latency, and robustness for real-world pick & place on a Franka FR3.
