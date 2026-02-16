# VLM Agent Helper Modules

Refactored VLM node with modular architecture.

## Structure

### `vlm_node.py` (main node)
- Main ROS node class
- Handles subscriptions, publishers, and callbacks
- Coordinates between helper modules

### `vlm_api_client.py`
- Ollama VLM API client
- Handles all communication with Ollama server
- Methods:
  - `locate_object()` - Find object and get pixel coordinates
  - `describe_scene()` - Get general scene description
  - `check_health()` - Verify Ollama connection

### `image_utils.py`
- Image processing utilities
- Functions:
  - `encode_image_to_base64()` - Convert image to base64
  - `draw_center_marker()` - Draw crosshair marker
  - `save_debug_image()` - Save annotated images
  - `parse_vlm_response()` - Parse VLM JSON responses

### `depth_utils.py`
- Depth image processing
- 3D position calculation using camera intrinsics
- Methods:
  - `get_3d_position()` - Convert pixel to 3D coordinates
  - `get_center_3d_position()` - Get 3D position of center pixel

## Configuration

All settings come from `/config.yaml`:
- VLM model name
- Ollama URL
- Temperature
- Timeout
- Camera topics
- Debug flags
- Image saving settings

## Usage

```bash
# Run the refactored node
ros2 run franka_vlm_agent vlm_node

# Request object localization
ros2 topic pub --once /vlm_request std_msgs/msg/String "{data: 'LOCATE red dice'}"

# Request scene description
ros2 topic pub --once /vlm_request std_msgs/msg/String "{data: 'DESCRIBE'}"
```

## Debug Images

When `save_images: true` in config, images are saved to:
```
/home/arash/franka-llm/debug_images/
```

Each image includes:
- Timestamp in filename
- Green crosshair marker at detected object center
- Pixel coordinates label
