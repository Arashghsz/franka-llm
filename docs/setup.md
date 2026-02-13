# System Setup Guide

## Prerequisites

- Ubuntu 22.04 LTS (for ROS2 Jazzy)
- Python 3.10+
- ROS2 Jazzy
- MoveIt2

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/Arashghsz/franka-multiagent-manipulation.git
cd franka-multiagent-manipulation
```

### 2. Create Virtual Environment (Optional)
```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 4. Build ROS2 Packages
```bash
colcon build --symlink-install
```

### 5. Source Setup
```bash
source install/setup.zsh
# or
source install/setup.bash
```

## Configuration

### Environment Variables
```bash
export FRANKA_ROBOT_IP=192.168.1.100
export FRANKA_USER=robot
export CAMERAS_NAMESPACE=/cameras/ee
```

### Launch File

Run the complete system:
```bash
ros2 launch franka_coordinator main.launch.py
```

Or individual components:
```bash
# Vision detection
ros2 launch franka_vision_detection table_detector.launch.py

# Robot state broadcasting
ros2 launch franka_robot_state_broadcaster state_broadcaster.launch.py

# LLM planner
python src/franka_llm_planner/franka_llm_planner/demo.py
```

## Verification

Check active topics:
```bash
ros2 topic list
```

Check robot state:
```bash
ros2 topic echo /franka_robot_state_broadcaster/robot_state
```

Monitor vision detections:
```bash
ros2 topic echo /detection/detections
```

## Troubleshooting

### Import Errors
- Ensure packages are built: `colcon build`
- Source setup: `source install/setup.zsh`

### Connection Issues
- Verify robot IP address
- Check firewall rules
- Ensure ROS2 domain ID matches

### Vision Issues
- Verify camera is connected
- Check RealSense driver: `realsense-viewer`
- Ensure calibration files are present

