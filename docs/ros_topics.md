# ROS2 Topics Reference

## Camera Topics (RealSense)

### Color Image
- `/cameras/ee/ee_camera/color/image_raw` - RGB image from end-effector camera
- `/cameras/ee/ee_camera/color/camera_info` - Camera intrinsic parameters

### Depth Image
- `/cameras/ee/ee_camera/depth/image_rect_raw` - Depth image (rectified)
- `/cameras/ee/ee_camera/depth/camera_info` - Depth camera parameters

### Aligned Depth
- `/cameras/ee/ee_camera/aligned_depth_to_color/image_raw` - Depth aligned to color frame
- `/cameras/ee/ee_camera/aligned_depth_to_color/camera_info`

### Metadata
- `/cameras/ee/ee_camera/color/metadata` - Color stream metadata
- `/cameras/ee/ee_camera/depth/metadata` - Depth stream metadata

## Detection Topics

### Table Detector
- `/detection/detections` - Object detections (YOLO output)
- `/detection/annotated_image` - Annotated image with detections
- `/detection/detections_debug` - Debug information
- `/detection/markers` - RViz visualization markers

## Robot State Topics

### Joint States
- `/franka/joint_states` - Franka arm joint positions/velocities
- `/fr3_gripper/joint_states` - Gripper joint state
- `/franka_robot_state_broadcaster/measured_joint_states` - Measured joint states
- `/franka_robot_state_broadcaster/desired_joint_states` - Desired joint states

### Pose Information
- `/franka_robot_state_broadcaster/current_pose` - Current end-effector pose
- `/franka_robot_state_broadcaster/last_desired_pose` - Last desired pose

### Force/Torque
- `/franka_robot_state_broadcaster/external_wrench_in_base_frame` - External forces/torques
- `/franka_robot_state_broadcaster/external_joint_torques` - Joint torques

### Robot State
- `/franka_robot_state_broadcaster/robot_state` - Complete robot state message

## Control Topics

### Trajectory Control
- `/fr3_arm_controller/joint_trajectory` - Joint trajectory command
- `/fr3_arm_controller/controller_state` - Controller state

### Gripper Control
- `/fr3_gripper/joint_states` - Gripper state

## Planning and Visualization

### MoveIt Planning
- `/planning_scene` - Current planning scene
- `/planning_scene_world` - World objects in planning scene
- `/monitored_planning_scene` - Monitored planning scene
- `/attached_collision_object` - Attached objects

### Goal and Control
- `/goal_pose` - Goal pose for motion planner
- `/initialpose` - Initial pose estimate

### Transform Topics
- `/tf` - Transforms between frames
- `/tf_static` - Static transforms

## System Topics

### ROS System
- `/rosout` - ROS logging output
- `/parameter_events` - Parameter change events
- `/diagnostics` - System diagnostics

### Visualization
- `/rviz_visual_tools` - RViz visual tools
- `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback`
- `/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update`

### Interactive Markers
- `/clicked_point` - User clicked point in RViz

## Controller Manager

- `/controller_manager/activity` - Controller manager activity
- `/controller_manager/introspection_data/*` - Controller introspection
- `/controller_manager/statistics/*` - Controller statistics

