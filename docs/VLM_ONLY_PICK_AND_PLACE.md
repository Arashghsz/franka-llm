# VLM-Only Pick & Place Architecture Report

**Date**: 2026-02-10 22:09:12  
**Author**: Arash (with Copilot analysis)  
**Status**: Proposal / Architecture Decision  

---

## Summary

This report proposes **dropping YOLO/Detectron2** from the pipeline and performing pick & place using **LLM + VLM only**, with depth from the existing RealSense RGB-D camera. This simplifies the architecture, reduces dependencies, and makes a stronger thesis contribution.

---

## Current Architecture (LLM → VLM → YOLO → Motion)

```
User Command → LLM (plan) → VLM (identify object) → YOLO (bbox + depth) → Motion (pick & place)
```

- YOLO provides **bounding box + depth** for precise localization
- VLM provides **semantic grounding** (text description only)
- This requires training/maintaining a separate object detector

## Proposed Architecture (LLM → VLM → Motion)

```
User Command → LLM (plan) → Grounding VLM (identify + locate) → Depth Lookup → Motion (pick & place)
```

- VLM provides **both** semantic understanding AND pixel-level localization
- Depth comes from **RealSense depth map** (already available)
- No YOLO, no Detectron2, no task-specific detector

---

## Why This Works

### The Core Insight

The only thing YOLO provides that VLM doesn't (currently) is a **bounding box in pixel coordinates**. Once you have a bounding box:

```
bbox centroid (u, v) → RealSense depth at (u, v) → 3D point (x, y, z) → MoveIt 2 goal
```

**A grounding-capable VLM can output bounding boxes natively**, replacing YOLO entirely.

### What the Current `vlm_node.py` Returns (Text Only)

```
"There's a red cup on the left side of the table, a green cube in the center..."
```

This is **not actionable** for motion planning — you can't convert "left side" to robot coordinates.

### What a Grounding VLM Returns (Bbox + Text)

```json
{
  "target": "red cup",
  "bbox": [234, 156, 310, 245],
  "rationale": "Red cup identified on the left side of the table"
}
```

This IS actionable: `bbox centroid → depth lookup → 3D coordinates → MoveIt 2`.

---

## Grounding VLM Options for Jetson AGX Orin

| Model | Parameters | Runs on Jetson? | Output Type | Spatial Accuracy | Notes |
|-------|-----------|:---------------:|-------------|:----------------:|-------|
| **Florence-2** | ~2B | ✅ Yes | Bbox coordinates | Good (~±10px) | Best balance of speed + accuracy |
| **Qwen-VL** | 7B | ✅ Via Ollama | `<box>x1,y1,x2,y2</box>` | Good | Already compatible with Ollama setup |
| **CogVLM** | 17B | ⚠️ Tight fit | Bbox coordinates | Very Good | May need quantization |
| **LLaVA 7B (current)** | 7B | ✅ Already running | Text only (can prompt-hack for coords) | Poor (~±50-100px) | Not recommended for localization |
| **Grounding DINO + SAM** | ~1B+~600M | ✅ Yes | Bbox + segmentation mask | Excellent | Two models, more complex integration |

### Recommendation: **Florence-2** or **Qwen-VL**

- **Florence-2**: Lightweight, fast, natively outputs bboxes via `<CAPTION_TO_PHRASE_GROUNDING>` task. Ideal for Jetson.
- **Qwen-VL**: Already works with Ollama (same setup as LLaVA). Returns bbox in structured format. Easier migration path.

---

## Proposed Simplified Pipeline

### System Architecture (3 nodes instead of 5)

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

### Depth Resolution Process (Replaces YOLO)

```python
# Pseudo-code for depth resolution from VLM bbox
def resolve_3d_position(bbox, depth_image, camera_intrinsics):
    """Convert VLM bounding box to 3D coordinates"""
    # 1. Get bbox centroid
    cx = (bbox[0] + bbox[2]) / 2  # center x pixel
    cy = (bbox[1] + bbox[3]) / 2  # center y pixel
    
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

## What Changes in the Codebase

### Modify: `franka_vlm_agent`
- Upgrade `vlm_node.py` to use a **grounding VLM** (Florence-2 or Qwen-VL)
- Change prompt from "describe the scene" to "locate the target object and return bounding box"
- Publish structured output (JSON with bbox + label + rationale) instead of plain text

### Modify: `franka_coordinator`
- Add **depth resolution** logic (bbox → depth → 3D point)
- Subscribe to `/vlm_grounding` (with bbox) instead of `/target_detection` (from YOLO)
- The rest of the orchestration stays the same

### Keep As-Is:
- `franka_llm_planner` — still generates the plan
- `franka_motion_executor` — still executes MoveIt 2 goals
- `realsense_cameras` — still publishes RGB-D

### Can Remove / Demote:
- `franka_vision_detection` (YOLO) — not needed in the core pipeline
  - Could keep as a **baseline comparison** for the thesis evaluation

---

## Impact on Thesis Evaluation

Your evaluation plan already includes comparing different configurations. This strengthens it:

| Configuration | What It Tests |
|--------------|---------------|
| **LLM + Grounding VLM** (proposed main) | Can edge VLM do grounding + localization for pick & place? |
| **YOLO-only** (baseline) | Traditional detection-based pick & place |
| **VLM + YOLO hybrid** (optional comparison) | Does adding YOLO on top of VLM improve anything? |

### Metrics to Report
- **Localization accuracy**: VLM bbox vs YOLO bbox (IoU comparison)
- **End-to-end success rate**: Pick & place completion
- **Latency**: VLM inference time on Jetson (~10-15s for LLaVA, ~2-3s for Florence-2)
- **Generalization**: Can VLM handle objects YOLO wasn't trained on?

---

## Thesis Contribution Statement

> *"We demonstrate that a Vision-Language Model running on edge hardware (Jetson AGX Orin) can perform both semantic understanding and spatial grounding for robotic manipulation — eliminating the need for task-specific object detectors (YOLO/Detectron2). Our distributed LLM+VLM architecture achieves pick & place with [X]% success rate while supporting open-vocabulary object references through natural language commands."*

---

## Next Steps

1. **Choose grounding VLM**: Test Florence-2 and Qwen-VL on Jetson AGX Orin
2. **Modify `vlm_node.py`**: Output bbox coordinates instead of text descriptions
3. **Add depth resolution**: Implement bbox → depth → 3D point in coordinator
4. **Test end-to-end**: LLM plan → VLM bbox → depth → MoveIt 2 pick & place
5. **Evaluate**: Compare VLM-only vs YOLO baseline on 3-5 pick & place scenarios

---

**Conclusion**: Dropping YOLO and using a grounding VLM is not only feasible — it makes a **cleaner architecture, a stronger thesis contribution, and a more novel system**. The key enabler is using a VLM that outputs bounding boxes (Florence-2 or Qwen-VL) combined with the RealSense depth camera you already have.