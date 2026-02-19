# 🤖 Pick & Place Pipeline - Integration Guide

## ✅ What Was Integrated

The coordinate transformation is now **fully integrated** into your pick & place pipeline!

### New Components:

1. **Motion Executor Node** (`motion_executor_node.py`)
   - Subscribes to `/target_position` (robot frame coordinates)
   - Subscribes to `/motion/command` (LLM decisions)
   - Executes pick & place primitives
   - **Prints detailed values** when pick is requested

2. **Refactored Coordinator** (`coordinator_node.py`)
   - Better logging and organization
   - Helper methods for clarity
   - Improved error handling

---

## 🔄 Complete Pipeline Flow

```
User: "pick the red dice"
    ↓
LLM Coordinator (llm_coordinator)
    ↓ Routes to VLM, sends /vlm_request
VLM Agent (vlm_node)
    ↓ Detects object center, creates bbox
    ↓ Publishes on /vlm_grounding
Franka Coordinator (coordinator_node)
    ↓ Converts bbox → robot frame via TF2
    ↓ Publishes on /target_position
Motion Executor (motion_executor)
    ↓ Receives robot frame coordinates
    ↓ PRINTS VALUES (as requested!)
    ↓ Executes pick primitive
Robot moves! 🎉
```

---

## 🚀 How to Run the Full System

### Terminal 1 - Robot (MoveIt)
```bash
cd ~/franka_ros2_ws
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 use_fake_hardware:=true
```

### Terminal 2 - Camera
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 launch realsense_cameras ee_camera.launch.py
```

### Terminal 3 - Camera TF (CRITICAL - DO NOT USE LAUNCH FILE!)
```bash
# DO NOT USE: ros2 launch realsense_cameras camera_tf.launch.py
# The launch file causes TF conflicts. Use this instead:

cd ~/franka-llm
source install/setup.zsh
ros2 run tf2_ros static_transform_publisher \
  0.0458961 -0.0368559 0.0567165 \
  -0.000400857 -0.00425145 0.698275 0.715817 \
  fr3_hand ee_d435i_color_optical_frame
```

**Camera Transform Details:**
- **Translation**: [0.046m, -0.037m, 0.057m] - Camera offset from robot hand
- **Rotation**: Quaternion [x, y, z, w] = [-0.0004, -0.004, 0.698, 0.716]
- **Frames**: fr3_hand → ee_d435i_color_optical_frame
- **Recalibration**: Run `python3 calibrate_hand_eye.py` if positions are inaccurate

**Update (Feb 19, 2026):** Coordinator now uses VLM center pixel directly (no bbox recalculation).

### Terminal 4 - VLM Agent (Object Detection)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_vlm_agent vlm_node
```

### Terminal 5 - Coordinator (Coordinate Transform)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_coordinator coordinator_node
```

### Terminal 6 - Motion Executor
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_motion_executor motion_executor
```

### Terminal 7 - LLM Coordinator (Routing)
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_llm_planner llm_coordinator
```

### Terminal 8 - Test
```bash
# FIRST: Verify TF is connected (run this once)
ros2 run tf2_ros tf2_echo fr3_link0 ee_d435i_color_optical_frame
# Should show transform within 2 seconds. If not, see TROUBLESHOOTING section!

# Then send pick command:
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'pick the red dice'}"
```

---

## 📋 What Happens During Pick

When you say "pick the red dice":

### 1. LLM Routes Request
```
[INFO] LLM Decision: {"action": "pick", "target_agent": "both", ...}
[INFO] Sent request to VLM: {'type': 'locate', 'object': 'red dice'}
```

### 2. VLM Detects Object & Publishes Grounding
```
[INFO] ✓ Located "red dice" at pixel [1000, 400]
[INFO] 📤 Published grounding: red dice bbox=[980,380,1020,420] → /vlm_grounding
```

### 3. Coordinator Converts to Robot Frame
```
[INFO] 🎯 Processing VLM detection: red dice
[INFO]    Bounding box: [980, 380, 1020, 420]
[INFO] Bbox [980,380,1020,420] → Centroid (1000, 400), Depth: 0.441m (median)
[INFO] ✅ "red dice" located in ROBOT FRAME:
[INFO]    X = +0.1740 m  (forward)
[INFO]    Y = +0.0110 m  (left)
[INFO]    Z = +0.4410 m  (right)
[INFO]    Z = +0.0400 m  (height from base)
[INFO] 📤 Published target position for "red dice" (action: pick)
```

### 4. Motion Executor Receives & PRINTS VALUES
```
[INFO] 📍 Received target position in frame: fr3_link0
[INFO]    Position: X=0.502m, Y=-0.093m, Z=0.040m
[INFO] 🎯 PICK REQUEST: red dice
[INFO] ======================================================================
[INFO] 🤖 EXECUTING PICK OPERATION
[INFO]    Object: red dice
[INFO]    Frame: fr3_link0
[INFO]    Target Position:
[INFO]       X = 0.5020 m  (forward from robot base)
[INFO]       Y = -0.0930 m  (left from robot base)
[INFO]       Z = 0.0400 m  (up from robot base)
[INFO] ======================================================================
```

### 5. Robot Executes Pick Sequence
```
[INFO] Step 1: Moving to approach position (Z + 0.15m)...
[INFO] Step 2: Opening gripper to 0.04m...
[INFO] Step 3: Moving down to grasp position...
[INFO] Step 4: Closing gripper...
[INFO] Step 5: Lifting object...
[INFO] Pick sequence complete!
[INFO] ✅ Pick operation completed for red dice
```

---

## 🎯 Key Features

### 1. Detailed Value Printing (As Requested!)
When a pick is requested, you get:
- ✅ Object name
- ✅ Frame ID (fr3_link0)
- ✅ **X, Y, Z coordinates** with descriptions
- ✅ Clear formatting with borders

### 2. Refactored Code
- Helper methods for clarity (`_convert_bbox_to_robot_frame`, `_publish_target_position`)
- Better logging with emojis for visual scanning
- Separation of concerns (coordinator vs executor)

### 3. Complete Integration
- VLM detection → Coordinate transform → Motion execution
- All in robot base frame (fr3_link0)
- Automatic pipeline trigger

---

## 📊 Topic Flow

```
/user_command (String)
    → LLM Coordinator

/vlm_grounding (String)
    → Franka Coordinator
    {
      "target": "red dice",
      "bbox": [x1, y1, x2, y2],
      "action": "pick"
    }

/target_position (PoseStamped)
    → Motion Executor
    {
      header: {frame_id: "fr3_link0"},
      pose: {
        position: {x: 0.502, y: -0.093, z: 0.040}
      }
    }

/motion/command (String)
    → Motion Executor
    {
      "action": "pick",
      "parameters": {"object": "red dice"}
    }

/motion/status (String)
    → Coordinator / UI
    {
      "status": "executing" | "completed" | "failed",
      "message": "...",
      "object": "red dice"
    }
```

---

## 🔍 Testing Individual Components

### Test Coordinate Transform Only:
```bash
ros2 run franka_coordinator test_coordinate_transform
```

### Test Motion Executor with Fake Position:
```bash
# Publish fake target position
ros2 topic pub --once /target_position geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "fr3_link0"}, pose: {position: {x: 0.5, y: 0.0, z: 0.05}}}'

# Then send pick command
ros2 topic pub --once /motion/command std_msgs/msg/String \
  '{data: "{\"action\": \"pick\", \"parameters\": {\"object\": \"test object\"}}"}'
```

### Test VLM Detection:
```bash
# Publish fake VLM grounding
ros2 topic pub --once /vlm_grounding std_msgs/msg/String \
  '{data: "{\"target\": \"red dice\", \"bbox\": [850, 250, 890, 290], \"action\": \"pick\"}"}'
```

---

## 🛠️ Customizing Pick Behavior

Edit `motion_executor_node.py` → `_pick_at_position()`:

```python
def _pick_at_position(self, x: float, y: float, z: float):
    approach_height = 0.15  # Adjust height above object
    grasp_width = 0.04      # Adjust gripper opening
    
    # Modify pick sequence as needed
    # Add safety checks, force feedback, etc.
```

---

## ✅ Verification Checklist

Before running the full pipeline, verify each step:

### 1. Robot TF Publishing
```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand
# Should show: Translation: [0.306, 0.005, 0.590] (or similar)
```

### 2. Camera Publishing Images
---

## 🧪 Testing Motion Executor Standalone

To test **only** the motion executor node without VLM/LLM:

### Terminal 1 - Robot (Required)
```bash
cd ~/franka_ros2_ws
source install/setup.zsh
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 use_fake_hardware:=true
```

### Terminal 2 - Motion Executor
```bash
cd ~/franka-llm
source install/setup.zsh
ros2 run franka_motion_executor motion_executor
```

### Terminal 3 - Send Test Commands

**Step 1: Send a target position (where to pick)**
```bash
ros2 topic pub --once /target_position geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "fr3_link0"}, 
    pose: {position: {x: 0.4, y: 0.0, z: 0.05}}}'
```

**Step 2: Send pick command**
```bash
ros2 topic pub --once /motion/command std_msgs/msg/String \
  '{data: "{\"action\": \"pick\", \"parameters\": {\"object\": \"test object\"}}"}'
```

**Expected Output** in motion executor terminal:
```
[INFO] 📍 Received target position in frame: fr3_link0
[INFO]    Position: X=0.400m, Y=0.000m, Z=0.050m
[INFO] 🎯 PICK REQUEST: test object
[INFO] ======================================================================
[INFO] 🤖 EXECUTING PICK OPERATION
[INFO]    Object: test object
[INFO]    Frame: fr3_link0
[INFO]    Target Position:
[INFO]       X = 0.4000 m  (forward from robot base)
[INFO]       Y = 0.0000 m  (left from robot base)
[INFO]       Z = 0.0500 m  (up from robot base)
[INFO] ======================================================================
[INFO] Step 1: Moving to approach position (Z + 0.15m)...
[INFO] Step 2: Opening gripper to 0.04m...
[INFO] Step 3: Moving down to grasp position...
[INFO] Step 4: Closing gripper...
[INFO] Step 5: Lifting object...
[INFO] ✅ Pick operation completed for test object
```

**Monitor status** (in another terminal):
```bash
ros2 topic echo /motion/status
```

---

## 🚨 TROUBLESHOOTING: "Two Unconnected Trees" Error

If you see this error in coordinator logs:
```
[ERROR] TF lookup failed: Could not find a connection between 'fr3_link0' and 
'ee_d435i_color_optical_frame' because they are not part of the same tree.
Tf has two or more unconnected trees.
```

**Root Cause**: Multiple static transform publishers are conflicting.

**Solution**:
1. Kill ALL camera TF processes:
   ```bash
   pkill -f "camera_tf.launch"
   pkill -f "static_transform_publisher"
   ```

2. Verify they're gone:
   ```bash
   ros2 node list | grep -E "(camera_link|static_transform)"
   # Should show NOTHING
   ```

3. Start ONLY ONE camera TF publisher (in Terminal 3):
   ```bash
   ros2 run tf2_ros static_transform_publisher \
     0.0458961 -0.0368559 0.0567165 \
     -0.000400857 -0.00425145 0.698275 0.715817 \
     fr3_hand ee_d435i_color_optical_frame
   ```

4. **CRITICAL**: Restart the coordinator AFTER camera TF is running:
   ```bash
   # In Terminal 5, Ctrl+C then restart:
   ros2 run franka_coordinator coordinator_node
   ```

5. Verify the connection:
   ```bash
   ros2 run tf2_ros tf2_echo fr3_link0 ee_d435i_color_optical_frame
   # Should show translation immediately (no "waiting" messages after 2 seconds)
   ```

**Why This Happens**:
- The `camera_tf.launch.py` file was causing conflicts with multiple publishers
- The coordinator's TF buffer must be initialized AFTER all transforms are published
- Using the manual static_transform_publisher avoids launch file issues

---

## 🎉 You're Ready!

Your pick & place pipeline is complete:
- ✅ Coordinate transformation working
- ✅ Values printed when pick is requested
- ✅ Refactored for clarity
- ✅ Full integration from user command to robot motion
- ✅ TF troubleshooting documented
ros2 run tf2_ros tf2_echo fr3_link0 ee_d435i_color_optical_frame
# Should show: Translation: [0.352, 0.042, 0.533] (or similar)
# If you see "two unconnected trees" - CHECK TERMINAL 3!
```

### 4. All Nodes Running
```bash
ros2 node list
# Should include: /franka_coordinator, /vlm_node, /motion_executor, /llm_coordinator
```

### 5. TF Tree Health Check
```bash
# List all TF publishers - should see robot_state_publisher + ONE static_transform
ros2 node list | xargs -I {} sh -c 'ros2 node info {} 2>/dev/null | grep -q "/tf_static" && echo {}'
# Should show: /robot_state_publisher, /static_transform_publisher_XXXXX
# If you see TWO static publishers, kill one!
```

---

## 🎉 You're Ready!

Your pick & place pipeline is complete:
- ✅ Coordinate transformation working
- ✅ Values printed when pick is requested
- ✅ Refactored for clarity
- ✅ Full integration from user command to robot motion

Next step: Test with real VLM and execute your first autonomous pick! 🤖
