# Test Full Pick Pipeline

## Quick Start (7 Terminals)

### Terminal 1 - Robot
```bash
cd ~/franka_ros2_ws
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false
```

### Terminal 2 - Camera
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 launch realsense_cameras ee_camera.launch.py
```

### Terminal 3 - Camera TF (IMPORTANT)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run tf2_ros static_transform_publisher 0.0458961 -0.0368559 0.0567165 -0.000400857 -0.00425145 0.698275 0.715817 fr3_hand ee_d435i_color_optical_frame
```

**Note:** This transform defines the camera position relative to the robot hand (fr3_hand).
- Translation: [0.046m, -0.037m, 0.057m] - Camera offset from hand
- Rotation: Quaternion [x, y, z, w] = [-0.0004, -0.004, 0.698, 0.716]
- If positions are inaccurate, recalibrate using: `python3 calibrate_hand_eye.py`

**Recent Update (Feb 19, 2026):** Coordinator now uses VLM center pixel directly instead of recalculating from bounding box.

### Terminal 4 - Coordinator (AFTER Terminal 3!)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_coordinator coordinator_node
```

### Terminal 5 - VLM Agent
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_vlm_agent vlm_node
```

### Terminal 6 - Motion Executor
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_motion_executor motion_executor
```

### Terminal 7 - LLM Coordinator (Optional for testing)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_llm_planner llm_coordinator
```

## Verify Everything is Connected

In a new terminal:
```bash
# Check TF connection (should show immediately, no waiting)
ros2 run tf2_ros tf2_echo fr3_link0 ee_d435i_color_optical_frame

# Check all nodes are running
ros2 node list
# Should show: /coordinator_node, /vlm_node, /motion_executor, /llm_coordinator
```

## Test Pick (Direct VLM Request)

**Method 1: Direct to VLM (Skip LLM for testing)**
```bash
ros2 topic pub --once /vlm_request std_msgs/msg/String \
  '{"data": "{\"type\": \"locate\", \"object\": \"red cup\", \"task\": \"pick\"}"}'
```

**Method 2: Through LLM (Full Pipeline)**
```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  '{data: "pick the red cup"}'
```

## Expected Output

### VLM Agent:
```
[INFO] Locating "red cup"...
[INFO] ✓ Located "red cup" at pixel [320, 240]
[INFO] 📤 Published grounding: red cup bbox=[300,220,340,260] → /vlm_grounding
```

### Coordinator:
```
[INFO] 🎯 Processing VLM detection: red cup
[INFO]    Bounding box: [300, 220, 340, 260]
[INFO] ✅ "red cup" located in ROBOT FRAME:
[INFO]    X = +0.4500 m  (forward)
[INFO]    Y = -0.0800 m  (right)
[INFO]    Z = +0.0400 m  (height from base)
[INFO] 📤 Published target position for "red cup" (action: pick)
[INFO] 📤 Sent motion command: pick "red cup"
```

### Motion Executor:
```
[INFO] Received target position: X=0.450m, Y=-0.080m, Z=0.040m
[INFO] Motion command: pick
[INFO] Pick request: red cup
[INFO] Executing pick: red cup
[INFO] Position: X=0.4500m, Y=-0.0800m, Z=0.0400m
[INFO] Step 1: Moving to position above object
[INFO] Step 2: Opening gripper
[INFO] Step 3: Moving down to grasp
[INFO] Step 4: Closing gripper
[INFO] Step 5: Moving to home
[INFO] Pick completed: red cup
```

## Troubleshooting

### If coordinator says "two unconnected trees":
1. Kill all TF publishers: `pkill -f static_transform_publisher`
2. Restart Terminal 3 (camera TF)
3. Restart Terminal 4 (coordinator)

### If VLM doesn't find object:
- Check camera is publishing: `ros2 topic hz /cameras/ee/ee_camera/color/image_raw`
- View camera image: `ros2 run rqt_image_view rqt_image_view`
- Make sure object is visible and well-lit

### If motion doesn't execute:
- Check motion executor received command: `ros2 topic echo /motion/command`
- Check target position was published: `ros2 topic echo /target_position`
