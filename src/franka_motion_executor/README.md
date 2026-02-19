# Franka Motion Executor

Executes pick and place motions based on target positions from VLM.

## Usage

Start the motion executor:
```bash
ros2 run franka_motion_executor motion_executor
```

### Pick Object

Send motion command:
```bash
ros2 topic pub --once /motion/command std_msgs/msg/String \
  '{data: "{\"action\": \"pick\", \"parameters\": {\"object\": \"cup\"}}"}'
```

Send target position:
```bash
ros2 topic pub --once /target_position geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "fr3_link0"}, pose: {position: {x: 0.6, y: -0.2, z: 0.2}}}'
```

### Place Object

Send motion command:
```bash
ros2 topic pub --once /motion/command std_msgs/msg/String \
  '{data: "{\"action\": \"place\", \"parameters\": {\"location\": \"table\"}}"}'
```

Send target position:
```bash
ros2 topic pub --once /target_position geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "fr3_link0"}, pose: {position: {x: 0.5, y: 0.3, z: 0.2}}}'
```

## Topics

**Subscribes:**
- `/motion/command` (String) - Pick/place commands with JSON format
- `/target_position` (PoseStamped) - Target coordinates in fr3_link0 frame

**Publishes:**
- `/motion/status` (String) - Execution status with JSON format

## Pick Sequence
1. Move to position above object (z=0.5m)
2. Open gripper (80mm)
3. Move down to grasp (z=0.14m)
4. Close gripper (30mm)
5. Return to home

## Place Sequence
1. Move to position above location (z=0.5m)
2. Move down to placement height (z=0.14m)
3. Open gripper to release (80mm)
4. Move back up (z=0.5m)
5. Return to home
