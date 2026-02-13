# Repository Renaming Rationale

**Date**: February 13, 2026  
**Previous Name**: `franka-llm`  
**New Name**: `franka-multiagent-manipulation`

## Why the Change?

### Problem with Previous Name
The name `franka-llm` was too narrow and focused only on the Language Model (LLM) component, which is just one part of a much larger and more sophisticated system.

### System Architecture
This is a **distributed multi-agent system** for robotic manipulation with the following components:

1. **LLM Node** (Task Planner) - Natural language task planning
2. **VLM Agent** (Vision Language Model) - Semantic grounding and scene understanding
3. **YOLO Detector** - Precise object localization
4. **Motion Executor** - MoveIt2-based motion planning and execution
5. **Coordinator** - Multi-agent orchestration and safety management
6. **RealSense Cameras** - RGB-D perception

### Deployment Architecture
- **Jetson AGX Orin (Edge AI)**: LLM + VLM agents
- **Controller PC**: Cameras, YOLO, Coordinator, Motion Execution, UI

## Why "franka-multiagent-manipulation"?

1. **Multi-Agent**: Reflects the distributed architecture with multiple specialized AI agents working together
2. **Manipulation**: Clearly identifies the application domain (pick & place, assembly tasks)
3. **Franka**: Maintains brand recognition for the Franka FR3 robot platform
4. **Professional**: Suitable for academic publication (Ro-Man 2026 conference)
5. **Descriptive**: Accurately represents the system's capabilities and innovation

## Alternative Names Considered
- `franka-distributed-vision` - Too focused on vision, doesn't capture full system
- `franka-edge-manipulation` - Emphasizes edge deployment but misses multi-agent aspect
- `franka-vlm-manipulation` - Better but still focused on single component
- `franka-intelligent-manipulation` - Too generic

## Impact
This is a **naming-only change**. No code functionality or architecture has been modified. The change affects:
- Documentation files (README, QUICKSTART, docs/)
- Package descriptions (package.xml, setup.py)
- UI branding (web dashboard)
- Path references in example commands
- Comments and docstrings

## Files Updated
19 files were modified to reflect the new repository name:
- All Markdown documentation
- ROS2 package metadata
- Python module docstrings
- HTML/JS UI files
- Example paths in quickstart guides

## For Developers
When cloning this repository:
```bash
git clone https://github.com/Arashghsz/franka-multiagent-manipulation.git
cd franka-multiagent-manipulation
```

All internal package names remain unchanged (e.g., `franka_llm_planner`, `franka_vlm_agent`) to avoid breaking existing ROS2 configurations and dependencies.
