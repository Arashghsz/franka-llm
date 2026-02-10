# 🤖 ROS 2 Learning Guide

## Setup
- **Platform**: Jetson AGX Orin (MAXN mode)
---

## 📚 Core Concepts

### 1. Nodes
Nodes are the basic building blocks - each is a small program that does ONE thing.

```bash
# See all running nodes
ros2 node list

# Get info about a specific node
ros2 node info /go2py_bridge
```

### 2. Topics
Topics are like "channels" where nodes publish and subscribe to data.

```bash
# List all topics
ros2 topic list

# See what type of message a topic uses
ros2 topic info /go2/imu

# Watch live data from a topic
ros2 topic echo /go2/imu

# See how fast messages are published
ros2 topic hz /go2/imu
```

### 3. Messages
Messages are the data structures. Each has a specific format.

```bash
# See the structure of a message type
ros2 interface show sensor_msgs/msg/Image
ros2 interface show geometry_msgs/msg/Twist
```

### 4. Services
Services are for request/response patterns (ask and get answer).

```bash
# List all services
ros2 service list

# Call a service
ros2 service call /service_name service_type "{request_data}"
```

### 5. Actions
Actions are for long-running tasks with feedback.

```bash
# List all actions
ros2 action list
```

---

## 🎮 Essential Commands Cheat Sheet

```bash
# === NODES ===
ros2 node list                    # List running nodes
ros2 node info <node_name>        # Details about a node

# === TOPICS ===
ros2 topic list                   # List all topics
ros2 topic echo <topic>           # View live data
ros2 topic hz <topic>             # Message frequency
ros2 topic info <topic>           # Topic details
ros2 topic pub <topic> <type> <data>  # Publish a message

# === SERVICES ===
ros2 service list                 # List services
ros2 service call <service> <type> <data>

# === PACKAGES ===
ros2 pkg list                     # List installed packages
ros2 pkg prefix <package>         # Find package location

# === LAUNCH ===
ros2 launch <package> <launch_file>  # Start a launch file

# === RUN A NODE ===
ros2 run <package> <executable>   # Run a single node
```

### Exercise 2: Visualize in RViz
```bash
# Start RViz (ROS visualization tool)
rviz2

# Add displays:
# - Image: /camera/color/image_raw
# - PointCloud2: /camera/depth/color/points
```

### Exercise 3: Control a Robot
```bash
# Publish velocity command (example for Go2)
ros2 topic pub /go2/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 📖 Learning Resources

### Official ROS 2 Tutorials
- https://docs.ros.org/en/humble/Tutorials.html

### Isaac ROS Documentation
- https://nvidia-isaac-ros.github.io/v/release-3.2/

### MoveIt 2 Tutorials
- https://moveit.picknik.ai/humble/

### YouTube Channels
- Articulated Robotics
- The Construct
- Robotics Back-End

---

## 💡 Tips

1. **Always source ROS 2**: Your zshrc does this automatically ✅
2. **Use tab completion**: Type partial commands and press Tab
3. **Use `--help`**: Most commands have help, e.g., `ros2 topic --help`
4. **Multiple terminals**: ROS 2 uses many terminals - use tmux or terminator
5. **Check errors first**: `ros2 doctor` can diagnose issues