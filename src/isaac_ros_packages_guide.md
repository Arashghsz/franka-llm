# 🎯 Isaac ROS Packages for Your Pick & Place Project

## Project Overview
You're building: **LLM + VLM Multi-Agent Pick & Place System**
- Robot: Franka FR3
- Camera: Intel RealSense RGB-D
- Goal: Natural language commands → Safe robotic manipulation

---

## 📦 Essential Packages for Your Project

### 🔴 **Critical** (Must Learn)

#### 1. **isaac_ros_cumotion** + **isaac_ros_cumotion_moveit**
**What it does**: GPU-accelerated motion planning for robot arms  
**Why you need it**: Makes your Franka arm move fast and safely  
**Key features**:
- 10-100x faster than traditional MoveIt planners
- Uses NVIDIA GPU on Jetson
- Collision avoidance built-in
- Integrates directly with MoveIt 2

**Documentation**: https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/manipulation/cumotion_moveit/index.html

```bash
# Example: Launch cuMotion with Franka
ros2 launch isaac_ros_cumotion_moveit cumotion_planner.launch.py
```

**Learning priority**: ⭐⭐⭐⭐⭐ (Start Week 5)

---

#### 2. **isaac_ros_nvblox**
**What it does**: Real-time 3D reconstruction from camera depth data  
**Why you need it**: Creates 3D map of workspace for obstacle avoidance  
**Key features**:
- Converts RealSense depth images → 3D voxel map
- Updates in real-time as objects move
- Exports collision data to MoveIt planning scene
- GPU-accelerated using NVIDIA

**Documentation**: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html

```bash
# Example: Launch nvblox with RealSense
ros2 launch nvblox_examples_bringup realsense_example.launch.py
```

**Learning priority**: ⭐⭐⭐⭐⭐ (Start Week 7)

---

#### 3. **isaac_ros_foundationpose**
**What it does**: 6D pose estimation (position + orientation) of objects  
**Why you need it**: Tells you WHERE objects are so robot can pick them  
**Key features**:
- Works with novel objects (no training needed!)
- Uses RGB-D input from RealSense
- Outputs 6D pose (x, y, z, roll, pitch, yaw)
- Track objects as they move

**Documentation**: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html

```bash
# Example: Estimate pose of an object
ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose.launch.py
```

**Learning priority**: ⭐⭐⭐⭐⭐ (Start Week 8)

---

### 🟡 **Important** (Very Useful)

#### 4. **isaac_ros_rtdetr** or **isaac_ros_yolov8**
**What it does**: Object detection (find objects in images)  
**Why you need it**: First step before pose estimation - "Is there a cup?"  
**Choose**:
- **RT-DETR**: Newer, very accurate, good for complex scenes
- **YOLOv8**: Faster, lighter, good for real-time

**Documentation**: 
- RT-DETR: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html
- YOLOv8: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_object_detection/isaac_ros_yolov8/index.html

**Learning priority**: ⭐⭐⭐⭐ (Start Week 8)

---

#### 5. **isaac_ros_image_pipeline**
**What it does**: Image processing (resize, rectify, debayer)  
**Why you need it**: Preprocess camera images before feeding to AI models  
**Key features**:
- GPU-accelerated image operations
- Camera calibration
- Image format conversions

**Learning priority**: ⭐⭐⭐ (Start Week 7)

---

### 🟢 **Nice to Have** (Advanced Features)

#### 6. **isaac_ros_visual_slam**
**What it does**: Camera-based localization and mapping  
**Why you might need it**: If robot or camera is moving, track position  
**Use case**: Mobile manipulation (if you combine Franka with Go2!)

**Learning priority**: ⭐⭐ (Optional, Week 12+)

---

#### 7. **isaac_ros_nitros**
**What it does**: High-performance message passing (zero-copy)  
**Why you care**: Makes everything faster - images don't need copying  
**Key features**:
- Automatic optimization
- Works transparently with other Isaac ROS packages
- Huge performance boost

**Note**: This is already built into Isaac ROS packages - you get it for free!

**Learning priority**: ⭐ (Automatic, no learning needed)

---

## 🔗 How These Packages Work Together

```
┌─────────────────────┐
│  RealSense Camera   │
│  (RGB-D Images)     │
└──────────┬──────────┘
           │
           ├──────────────────────────────────┐
           │                                  │
           ▼                                  ▼
┌─────────────────────┐          ┌─────────────────────┐
│  isaac_ros_nvblox   │          │ isaac_ros_rtdetr    │
│  (3D Reconstruction)│          │ (Object Detection)  │
└──────────┬──────────┘          └──────────┬──────────┘
           │                                │
           │                                ▼
           │                     ┌─────────────────────┐
           │                     │ isaac_ros_          │
           │                     │ foundationpose      │
           │                     │ (6D Pose)           │
           │                     └──────────┬──────────┘
           │                                │
           │     ┌──────────────────────────┘
           │     │
           ▼     ▼
    ┌──────────────────────┐
    │   MoveIt 2           │
    │   Planning Scene     │
    │   (Obstacles + Goal) │
    └──────────┬───────────┘
               │
               ▼
    ┌──────────────────────┐
    │  isaac_ros_cumotion  │
    │  (Motion Planning)   │
    └──────────┬───────────┘
               │
               ▼
    ┌──────────────────────┐
    │   Franka FR3 Robot   │
    │   (Execute Motion)   │
    └──────────────────────┘
```

---

## 🎓 Recommended Learning Order

### Week 1-2: ROS 2 Basics
*Before Isaac ROS packages*
- Topics, nodes, services (see [ros2_learning_guide.md](ros2_learning_guide.md))

### Week 3-4: Simulation
- Gazebo with Franka
- MoveIt basics

### Week 5-6: Motion Planning
**Start here with Isaac ROS!**
1. Install `isaac_ros_cumotion`
2. Launch cuMotion with Franka in simulation
3. Compare speed vs. default MoveIt planners
4. Learn cuMotion configuration

**Exercise**: Move Franka to different poses using cuMotion

---

### Week 7-8: Perception Pipeline
**Camera & 3D Mapping**
1. Launch RealSense camera
2. Install and run `isaac_ros_nvblox`
3. Visualize 3D reconstruction in RViz
4. Connect nvblox to MoveIt planning scene

**Exercise**: Place object in workspace, see it in nvblox map

**Object Detection**
1. Install `isaac_ros_rtdetr` or `isaac_ros_yolov8`
2. Run detection on camera feed
3. Train on your objects (cups, boxes, etc.)

**Exercise**: Detect objects in your workspace

---

### Week 9-10: Pose Estimation
1. Install `isaac_ros_foundationpose`
2. Get 6D pose of detected objects
3. Transform poses to robot base frame
4. Send to MoveIt as grasp targets

**Exercise**: Detect object + get pose + plan motion to it

---

### Week 11-12: LLM + VLM Integration
- Build your custom ROS 2 nodes
- Integrate LLM for task planning
- Add VLM for safety checks

---

## 📥 Installation Guide

### Prerequisites
```bash
# Make sure ROS 2 Humble is installed
source /opt/ros/humble/setup.zsh

# Your workspace
cd ~/workspaces/isaac_ros-dev
```

### Install Isaac ROS Common (Base Package)
```bash
cd ~/workspaces/isaac_ros-dev/src

# Clone Isaac ROS Common (if not already done)
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Build
cd ~/workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_common
source install/setup.zsh
```

### Install cuMotion (Priority #1)
```bash
cd ~/workspaces/isaac_ros-dev/src

# Clone cuMotion
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git

# Build
cd ~/workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_cumotion isaac_ros_cumotion_moveit
source install/setup.zsh
```

### Install Nvblox (Priority #2)
```bash
cd ~/workspaces/isaac_ros-dev/src

# Clone nvblox
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git

# Build
cd ~/workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_nvblox
source install/setup.zsh
```

### Install Object Detection
```bash
cd ~/workspaces/isaac_ros-dev/src

# Clone object detection
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git

# Build (RT-DETR)
cd ~/workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_rtdetr
source install/setup.zsh
```

### Install FoundationPose
```bash
cd ~/workspaces/isaac_ros-dev/src

# Clone pose estimation
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git

# Build
cd ~/workspaces/isaac_ros-dev
colcon build --packages-select isaac_ros_foundationpose
source install/setup.zsh
```

---

## 🧪 Quick Test Commands

### Test cuMotion
```bash
# Launch cuMotion demo
ros2 launch isaac_ros_cumotion_examples franka_isaac_sim.launch.py
```

### Test Nvblox
```bash
# Launch with RealSense
ros2 launch nvblox_examples_bringup realsense_example.launch.py

# Visualize in RViz
rviz2 -d $(ros2 pkg prefix nvblox_examples_bringup)/share/nvblox_examples_bringup/config/nvblox_example.rviz
```

### Test Object Detection
```bash
# Launch RT-DETR
ros2 launch isaac_ros_rtdetr isaac_ros_rtdetr.launch.py
```

---

## 📚 Key Documentation Links

### Official Isaac ROS
- **Main Docs**: https://nvidia-isaac-ros.github.io/v/release-3.2/
- **Isaac Manipulator (Your Use Case!)**: https://nvidia-isaac-ros.github.io/v/release-3.2/reference_workflows/isaac_manipulator/index.html

### Package-Specific
- **cuMotion**: https://nvidia-isaac-ros.github.io/v/release-3.2/concepts/manipulation/cumotion_moveit/index.html
- **Nvblox**: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html
- **FoundationPose**: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html

---

## ✅ cuMotion Installation Complete!

**Status**: cuMotion packages are installed in Isaac ROS Dev Docker container!

Installed packages:
- ✅ `ros-humble-isaac-ros-cumotion` (core planner)
- ✅ `ros-humble-isaac-ros-cumotion-moveit` (MoveIt plugin)
- ✅ `ros-humble-isaac-ros-cumotion-interfaces`
- ✅ `ros-humble-isaac-ros-cumotion-robot-description`

**How to use Docker environment**:
```bash
# From host machine, enter Docker:
cd ~/workspaces/isaac_ros-dev
src/isaac_ros_common/scripts/run_dev.sh

# Inside Docker (admin@ubuntu prompt):
source /opt/ros/humble/setup.bash
```

---

## 🎯 Your Next Steps

### Today (Week 1):
1. ✅ cuMotion installed in Docker! 
2. ✅ Docker environment working
3. ⬜ Read `ros2_learning_guide.md` for ROS 2 basics
4. ⬜ Learn Docker workflow (enter/exit container)

### This Week (ROS 2 Basics):
1. ⬜ Complete ROS 2 basic tutorials
2. ⬜ Launch Franka in Gazebo simulation  
3. ⬜ Test MoveIt with Franka
4. ⬜ Understand MoveIt Python API

### Next Week (cuMotion Week!):
1. ✅ cuMotion already installed!
2. ⬜ Follow NVIDIA Franka tutorial: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html
3. ⬜ Compare cuMotion vs. standard planners
4. ⬜ Configure cuMotion for your Franka

---

## 💡 Pro Tips

1. **Use Docker**: Isaac ROS provides Docker containers with everything pre-installed
   - Easier than manual installation
   - All dependencies handled
   - See: https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/dev_env_setup.html

2. **Start Small**: Don't try to use all packages at once
   - Week 5-6: Just cuMotion
   - Week 7: Add nvblox
   - Week 8: Add object detection

3. **Test in Simulation First**: 
   - Cheaper mistakes
   - Faster iteration
   - No hardware damage

4. **Check Isaac Manipulator Tutorial**:
   - NVIDIA has a complete pick & place tutorial
   - Uses same hardware (Franka + RealSense)
   - Perfect reference for your project
   - Link: https://nvidia-isaac-ros.github.io/v/release-3.2/reference_workflows/isaac_manipulator/index.html

---

## 🤔 Questions?

Common questions:

**Q: Do I need all these packages?**  
A: No! Start with cuMotion, then add others as needed.

**Q: Can I run this on Jetson?**  
A: Yes! All Isaac ROS packages are optimized for Jetson.

**Q: What about RealSense drivers?**  
A: Use `realsense2_camera` ROS 2 package (separate from Isaac ROS).

**Q: Docker or native installation?**  
A: Docker is easier. Native gives more control. Your choice!

---

Ready to start learning? Go back to [LEARNING_PATH.md](LEARNING_PATH.md) for week-by-week exercises! 🚀
