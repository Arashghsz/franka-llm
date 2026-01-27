# 🚀 Learning Path: LLM + VLM Multi-Agent Pick & Place

## Your Goal
Build a multi-agent robotic system:
- **LLM** → Task Planner (high-level reasoning)
- **VLM/VLA** → Safety Agent (visual verification)
- **cuMotion** → Motion Planning (GPU-accelerated)
- **Franka FR3** → Execution

---

## 📅 Learning Phases

### Phase 1: ROS 2 Fundamentals (Week 1-2)
**Goal**: Understand how ROS 2 works

| Day | Topic | Exercise |
|-----|-------|----------|
| 1 | Nodes, Topics, Messages | Run `ros2 topic echo`, create simple publisher |
| 2 | Services & Actions | Call services, understand MoveIt actions |
| 3 | Launch files | Write launch files in Python |
| 4 | TF2 (Transforms) | Understand robot coordinate frames |
| 5 | RViz2 | Visualize robots and sensors |
| 6 | Colcon & Packages | Build your first ROS 2 package |
| 7 | Review & Practice | |

**Resources**:
- https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html
- https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html

---

### Phase 2: Simulation (Week 3-4)
**Goal**: Test without hardware

| Option | Pros | Cons |
|--------|------|------|
| **Isaac Sim** | Best for Isaac ROS, GPU rendering | Heavy, needs good GPU |
| **Gazebo** | Lightweight, widely used | Less realistic physics |
| **MuJoCo** | Fast, accurate physics | Less ROS integration |

We'll use **Gazebo** for learning (lighter) and **Isaac Sim** for final testing.

```bash
# Franka in Gazebo (you have this!)
ros2 launch franka_gazebo_bringup gazebo.launch.py
```

---

### Phase 3: MoveIt 2 & Motion Planning (Week 5-6)
**Goal**: Control robot arm programmatically

Topics:
- MoveIt concepts (planning scene, collision objects)
- Python MoveIt interface
- Adding obstacles from camera
- Grasp planning

```python
# Example: Move to pose
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="my_moveit")
arm = moveit.get_planning_component("panda_arm")
arm.set_goal_state(pose_goal)
arm.plan_and_execute()
```

---

### Phase 4: Perception Pipeline (Week 7-8)
**Goal**: See and understand the world

| Component | Purpose |
|-----------|---------|
| RealSense Camera | RGB + Depth |
| Isaac ROS NVBLOX | 3D reconstruction |
| Isaac ROS Object Detection | Find objects |
| FoundationPose | 6D object pose |

Pipeline:
```
Camera → NVBLOX → 3D Scene → MoveIt Collision World
         ↓
       Object Detection → Object Poses → Grasp Targets
```

---

### Phase 5: LLM Integration (Week 9-10)
**Goal**: Natural language to robot actions

**Architecture Options**:

1. **Direct Prompting** (Simple)
```
User: "Pick up the red cup"
LLM → Parse → {"action": "pick", "object": "red cup"}
```

2. **Code Generation** (More flexible)
```
User: "Pick up the red cup and place it on the shelf"
LLM → Generate Python code → Execute
```

3. **ReAct Pattern** (Reasoning + Acting)
```
LLM thinks: "I need to locate the red cup first"
LLM acts: call_perception("find red cup")
LLM observes: "Red cup at position (0.5, 0.2, 0.1)"
LLM thinks: "Now I can pick it"
LLM acts: call_motion("pick", position)
```

**LLM Options**:
- OpenAI GPT-4 (API)
- Claude (API)
- LLaMA 3 (local on Jetson!)
- Ollama (easy local deployment)

---

### Phase 6: VLM Safety Agent (Week 11-12)
**Goal**: Visual verification before actions

**Role of VLM**:
- Verify scene matches LLM's understanding
- Check for obstacles/hazards
- Confirm grasp feasibility
- Monitor execution

**VLM Options for Jetson**:
- LLaVA (can run locally)
- VILA (NVIDIA's VLM)
- GPT-4V (API)

**Safety Checks**:
```python
# Before executing motion
def safety_check(image, planned_action):
    prompt = f"""
    Image shows the robot workspace.
    Planned action: {planned_action}
    
    Is this safe to execute? Check for:
    1. Obstacles in path
    2. Human presence
    3. Object still in expected position
    
    Respond: SAFE or UNSAFE with reason
    """
    return vlm.query(image, prompt)
```

---

## 🛠️ Simulation Setup (No Hardware Needed)

### Option 1: Gazebo (Lighter, good for learning)
```bash
# Launch Franka in Gazebo
ros2 launch franka_gazebo_bringup gazebo.launch.py

# In another terminal, launch MoveIt
ros2 launch franka_fr3_moveit_config moveit.launch.py
```

### Option 2: Isaac Sim (When ready for realistic testing)
- Download: https://developer.nvidia.com/isaac-sim
- Requires x86 workstation with RTX GPU
- Can connect to Jetson for ROS 2

---

## 📁 Project Structure (We'll Build This)

```
~/workspaces/isaac_ros-dev/src/
├── franka_llm_planner/          # LLM task planning
│   ├── franka_llm_planner/
│   │   ├── __init__.py
│   │   ├── llm_node.py          # ROS 2 node for LLM
│   │   ├── task_parser.py       # Parse LLM output
│   │   └── prompts.py           # Prompt templates
│   ├── launch/
│   ├── config/
│   └── package.xml
│
├── franka_vlm_safety/           # VLM safety agent
│   ├── franka_vlm_safety/
│   │   ├── __init__.py
│   │   ├── vlm_node.py          # ROS 2 node for VLM
│   │   └── safety_checks.py
│   └── ...
│
├── franka_pick_place/           # Pick & place logic
│   ├── franka_pick_place/
│   │   ├── __init__.py
│   │   ├── coordinator.py       # Orchestrates everything
│   │   ├── perception.py        # Object detection
│   │   └── manipulation.py      # Motion execution
│   └── ...
│
└── ros2_learning_guide.md       # This guide
```

---

## 🎮 Today's Exercises (No Hardware)

### Exercise 1: Explore Gazebo Simulation
```bash
# Terminal 1: Launch Gazebo with Franka
ros2 launch franka_gazebo_bringup gazebo.launch.py

# Terminal 2: Check topics
ros2 topic list | grep franka

# Terminal 3: Launch MoveIt
ros2 launch franka_fr3_moveit_config moveit.launch.py
```

### Exercise 2: Create Your First ROS 2 Package
```bash
cd ~/workspaces/isaac_ros-dev/src
ros2 pkg create --build-type ament_python franka_llm_planner
```

### Exercise 3: Write a Simple Publisher
Create a node that publishes text commands.

---

## 📚 Key Resources

### ROS 2
- [Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [The Construct (YouTube)](https://www.youtube.com/c/TheConstruct)

### MoveIt 2
- [MoveIt Tutorials](https://moveit.picknik.ai/humble/)
- [MoveIt Python API](https://moveit.picknik.ai/humble/doc/examples/moveit_py/moveit_py_tutorial.html)

### LLM + Robotics
- [SayCan Paper](https://say-can.github.io/) - LLM + Affordances
- [Code as Policies](https://code-as-policies.github.io/) - LLM → Code → Robot
- [RT-2](https://robotics-transformer2.github.io/) - Vision-Language-Action

### Isaac ROS
- [Isaac Manipulator](https://nvidia-isaac-ros.github.io/v/release-3.2/reference_workflows/isaac_manipulator/index.html)
- [cuMotion Tutorial](https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/manipulation/cumotion_moveit/index.html)

---

## 🎯 Milestones

- [ ] **M1**: Run Franka in Gazebo simulation
- [ ] **M2**: Move robot with MoveIt Python API
- [ ] **M3**: Create custom ROS 2 package
- [ ] **M4**: Integrate camera in simulation
- [ ] **M5**: Object detection working
- [ ] **M6**: Basic pick & place in simulation
- [ ] **M7**: LLM parsing natural language commands
- [ ] **M8**: VLM safety verification
- [ ] **M9**: Full pipeline in simulation
- [ ] **M10**: Real robot execution

---

## Next Step
Let's start with **Exercise 1** - launching Gazebo simulation!
