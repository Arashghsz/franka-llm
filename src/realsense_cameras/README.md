# RealSense Cameras for Franka Multi-Agent Manipulation

Multi-camera streaming package for the Franka robot using RealSense sensors.

## Hardware Setup

- **Fixed Camera**: Intel RealSense D415 (workspace view)
  - Serial: 821312061433
  - Resolution: 640×480 @ 30 FPS
  - Streams: RGB, Depth, IR

- **End-Effector Camera**: Intel Real
Sense D435i (gripper mount)
  - Serial: 845112071321
  - Resolution: 640×480 @ 30 FPS
  - Streams: RGB, Depth, IR

## Installation

### Dependencies
```bash
sudo apt-get install ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs librealsense2-tools
```

### Build
```bash
cd ~/franka-multiagent-manipulation
colcon build --packages-select realsense_cameras
source install/setup.zsh
```

## Usage

### Launch All Cameras
```bash
ros2 launch realsense_cameras cameras.launch.py
```

### Launch Fixed Camera Only
```bash
ros2 launch realsense_cameras fixed_camera.launch.py
```

### Launch End-Effector Camera Only
```bash
ros2 launch realsense_cameras ee_camera.launch.py
```

### Check Connected Cameras
```bash
ros2 run realsense_cameras camera_hub
```

## Topic Structure

### Fixed Camera
- `/cameras/fixed/fixed_camera/color/image_raw` - RGB image
- `/cameras/fixed/fixed_camera/depth/image_rect_raw` - Depth map
- `/cameras/fixed/fixed_camera/infra1/image_rect_raw` - IR stream 1
- `/cameras/fixed/fixed_camera/aligned_depth_to_color/*` - Aligned depth

### End-Effector Camera
- `/cameras/ee/ee_camera/color/image_raw` - RGB image
- `/cameras/ee/ee_camera/depth/image_rect_raw` - Depth map
- `/cameras/ee/ee_camera/infra1/image_rect_raw` - IR stream 1
- `/cameras/ee/ee_camera/infra2/image_rect_raw` - IR stream 2

### Frame Broadcasting
- `/tf` - Dynamic transforms
- `/tf_static` - Static transforms (camera extrinsics)

## Configuration

All camera parameters are defined in the launch files:
- **Color Profile**: 640×480 @ 30 FPS
- **Depth Profile**: 640×480 @ 30 FPS
- **Depth Alignment**: Enabled (aligned to RGB)
- **Temporal Filtering**: Enabled
- **Frame Synchronization**: Enabled for D435i

## Visualization

```bash
rviz2
```

Add displays:
1. Subscribe to `/cameras/fixed/fixed_camera/color/image_raw`
2. Subscribe to `/cameras/ee/ee_camera/color/image_raw`
3. Add TF to see camera frames

## Troubleshooting

### Camera Not Detected
```bash
# Check USB permissions
ls -la /dev/bus/usb/*/
# Verify camera firmware
rs-enumerate-devices -d
```

### No Frame Output
- Check USB port stability
- Verify depth alignment is working
- Check temporal filter settings

## Next Steps

- Frame calibration between fixed and end-effector cameras
- Extrinsic calibration setup
- Perception pipeline integration

## License
- TBD
