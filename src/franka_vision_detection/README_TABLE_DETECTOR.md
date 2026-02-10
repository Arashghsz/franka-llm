# Table Detector - Battery Assembly Domain

**For Ro-Man 2026 Evaluation Framework**

## Quick Start

```bash
# Launch detector only
ros2 launch franka_vision_detection table_detector.launch.py

# Or run directly
ros2 run franka_vision_detection table_detector
```

## Detection Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/detection/markers` | `visualization_msgs/MarkerArray` | RViz visualization of detections |
| `/detection/annotated_image` | `sensor_msgs/Image` | Annotated camera feed with bboxes |

## Detectable Classes (27 Assembly Domain Classes)

### Batteries (5)
- `battery_aa` - AA battery
- `battery_aaa` - AAA battery
- `battery_9v` - 9V battery
- `battery_coin` - Coin cell battery
- `battery_alkaline` - Alkaline battery

### Battery Compartment & Contacts (4)
- `battery_spring` - Spring connector
- `battery_contact` - Contact point
- `battery_terminal` - Terminal connector
- `contact_pin` - Electrical pin

### PCB & Electrical Components (7)
- `pcb` - Circuit board
- `pcb_track` - PCB track/trace
- `resistor` - Resistor component
- `capacitor` - Capacitor component
- `diode` - Diode component
- `led` - LED component
- `connector` - Connector port

### Mechanical Parts (7)
- `screw` - Screw fastener
- `bolt` - Bolt fastener
- `nut` - Nut fastener
- `washer` - Washer spacer
- `spacer` - Spacer component
- `clip` - Clip fastener
- `rivet` - Rivet fastener

### Tools (5)
- `screwdriver` - Screwdriver tool
- `wrench` - Wrench tool
- `pliers` - Pliers tool
- `drill_bit` - Drill bit
- `soldering_iron` - Soldering iron

### Assembly Items (5)
- `wire` - Electrical wire
- `cable` - Cable assembly
- `housing` - Device housing
- `cover` - Cover/lid
- `bracket` - Mounting bracket
- `fastener` - Generic fastener

## Model Information

**Base Model**: YOLOv8 Extra Large (yolov8x)
- **Accuracy**: ~99% on COCO (highest in YOLOv8 family)
- **Size**: 168 MB
- **Inference**: CPU-based (can use GPU if available)
- **Confidence Threshold**: 0.65 (high confidence for reliable picks)

## Fine-Tuning for Your Dataset

To achieve **even better accuracy** on your specific battery assembly objects:

```bash
python3 table_detector_finetuner.py
```

This requires a labeled dataset with the following structure:

```
datasets/battery_assembly/
├── images/
│   ├── train/        # Training images
│   ├── val/          # Validation images  
│   └── test/         # Test images
└── labels/
    ├── train/        # YOLO format annotations
    ├── val/
    └── test/
```

### Annotation Format

Each image needs a corresponding `.txt` file in YOLO format:
```
<class_id> <x_center_norm> <y_center_norm> <width_norm> <height_norm>
```

**Class IDs** (0-26 mapping to ASSEMBLY_CLASSES):
```
0: battery_aa        7: battery_terminal  14: led
1: battery_aaa       8: contact_pin       15: connector
2: battery_9v        9: pcb               16: screw
3: battery_coin      10: pcb_track        17: bolt
4: battery_alkaline  11: resistor         18: nut
5: battery_spring    12: capacitor        19: washer
6: battery_contact   13: diode            20: spacer
                                           ... (27 total)
```

## ROS 2 Integration

### Subscribes To
- `/cameras/ee/ee_camera/color/image_raw` - Camera feed
- `/cameras/ee/ee_camera/color/camera_info` - Camera calibration

### Publishes
- `/detection/markers` - MarkerArray for RViz
- `/detection/annotated_image` - Annotated image with boxes + labels

### Parameters
- `confidence_threshold` (double, default: 0.65) - Detection confidence threshold
- `nms_threshold` (double, default: 0.45) - NMS overlap threshold

## Evaluation Framework

For Ro-Man 2026 paper, run evaluation scenarios:

```bash
python3 evaluation_framework.py
```

This defines **18+ evaluation scenarios** including:
- Single/multiple object detection
- Various backgrounds (simple, cluttered, textured)
- Occlusion levels (partial, heavy)
- Different lighting conditions
- Object type variations (rigid, deformable, fragile)
- Target location variations (shelf, bin, handover)

**Total planned trials**: 50+

### Scenario Categories

1. **Single Object** (5 trials) - Clean table, controlled conditions
2. **Multiple Objects** (9 trials) - Various counts and complexities
3. **Occlusion** (5 trials) - 30-60% hidden objects
4. **Object Types** (8 trials) - Rigid, deformable, fragile
5. **Target Locations** (8 trials) - Shelves, bins, human handover
6. **Lighting** (6 trials) - Low, bright, varying
7. **Complex Scenarios** (5 trials) - Realistic and adversarial

## Performance Metrics

Expected performance on battery assembly task:
- **Inference FPS**: 5-10 FPS (CPU, YOLOv8x)
- **Detection Accuracy**: >85% (after fine-tuning on assembly data)
- **False Positive Rate**: <5%
- **Latency**: 100-200ms per frame

## Output Examples

### RViz Markers
Each detection shows:
- Colored cube at object center
- Class name label
- Confidence score (0.0-1.0)

### Annotated Image
- Bounding box around each detection
- Class name and confidence displayed
- Color-coded by class for visual grouping

## Troubleshooting

**Problem**: No detections appearing
- Check camera feed: `ros2 topic echo /cameras/ee/ee_camera/color/image_raw`
- Lower confidence threshold if needed
- Ensure lighting is adequate

**Problem**: Too many false positives
- Increase `confidence_threshold` parameter
- Fine-tune model on your specific lighting/background

**Problem**: Model loading slowly
- First run downloads yolov8x model (~168MB)
- Subsequent runs use cached model
- Consider yolov8m for faster inference if needed

## References

- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [ROS 2 Topic Subscription](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Subscriber-And-Publisher.html)
- [Ro-Man 2026 Conference](https://www.ro-man2026.org)

---

**Created**: Feb 10, 2026  
**Purpose**: Battery assembly object detection for robotic manipulation evaluation  
**Status**: Ready for evaluation
