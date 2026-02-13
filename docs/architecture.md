# Franka Multi-Agent Manipulation System Architecture

## Pipeline Overview

The Franka-LLM system follows a modular pipeline architecture:

```
User Input → LLM Planner → VLM Agent → Vision Detection (YOLO) → Motion Executor → Robot
```

## Components

### 1. LLM Planner (`franka_llm_planner`)
- High-level task planning from natural language
- Generates action sequences

### 2. VLM Agent (`franka_vlm_agent`)
- Vision Language Model for scene understanding
- Processes camera input and provides semantic context

### 3. Vision Detection (`franka_vision_detection`)
- YOLO-based object detection
- Table detection and scene analysis

### 4. Motion Executor (`franka_motion_executor`)
- Executes planned motions using MoveIt2
- Handles gripper control and trajectory execution

### 5. Coordinator (`franka_coordinator`)
- Orchestrates communication between pipeline stages
- Manages data flow and synchronization

### 6. Conversation Logger (`franka_conversation_logger`)
- Logs all interactions and conversations
- Maintains conversation history

### 7. Evaluation Tools (`franka_eval_tools`)
- Performance metrics and evaluation
- Task success/failure analysis

## Message/Service Interface (`franka_interfaces`)
- Centralized message and service definitions
- Common data structures for inter-node communication

## Supporting Packages

### Realsense Cameras (`realsense_cameras`)
- Intel RealSense camera drivers and publishers
- Provides depth and RGB streams

## Data Flow

1. **Input**: Natural language commands from UI or console
2. **Planning**: LLM processes request and creates task plan
3. **Perception**: VLM analyzes scene, YOLO detects objects
4. **Execution**: Motion executor carries out planned actions
5. **Logging**: All interactions recorded for evaluation
6. **Feedback**: Results reported back to user

