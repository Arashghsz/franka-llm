#!/usr/bin/env python3
"""
Table Detector - Single file for detecting objects on table for evaluation
Uses YOLOv8 Extra Large for maximum accuracy across diverse object types
Publishes detections as RViz markers + annotated images
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import List, Dict, Tuple
import os
import time


class TableDetector(Node):
    """Detects objects on table for pick & place evaluation"""
    
    # COCO classes (80 total) - standard object detection
    COCO_CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
        'boat', 'traffic_light', 'fire_hydrant', 'stop_sign', 'parking_meter', 'bench',
        'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
        'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis',
        'snowboard', 'sports_ball', 'kite', 'baseball_bat', 'baseball_glove', 'skateboard',
        'surfboard', 'tennis_racket', 'bottle', 'wine_glass', 'cup', 'fork', 'knife',
        'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
        'hot_dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted_plant', 'bed',
        'dining_table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
        'teddy_bear', 'hair_drier', 'toothbrush'
    ]
    
    # Color palette for visualization (cycling through colors)
    COLORS = [
        (0, 255, 0),      # Green
        (255, 0, 0),      # Blue
        (0, 255, 255),    # Cyan
        (255, 255, 0),    # Yellow
        (255, 0, 255),    # Magenta
        (128, 0, 255),    # Purple
        (255, 165, 0),    # Orange
        (0, 128, 255),    # Red-Orange
        (255, 192, 203),  # Pink
        (0, 255, 127),    # Spring Green
    ]
    
    def __init__(self):
        super().__init__('table_detector')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Initializing Table Detector using YOLOv8 Extra Large")
        self.get_logger().info("=" * 60)
        
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.detector = None
        self.load_detector()
        
        if not self.detector:
            self.get_logger().fatal("Failed to load detector. Exiting.")
            raise RuntimeError("Detector initialization failed")
        
        # Camera intrinsics
        self.camera_matrix = None
        self.frame_id = None
        
        # Publishers
        self.markers_pub = self.create_publisher(MarkerArray, '/detection/markers', 10)
        self.annotated_pub = self.create_publisher(Image, '/detection/annotated_image', 10)
        self.detections_pub = self.create_publisher(Image, '/detection/detections_debug', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/cameras/ee/ee_camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cameras/ee/ee_camera/color/camera_info',
            self.camera_info_callback,
            1
        )
        
        # Parameters
        self.confidence_threshold = self.declare_parameter('confidence_threshold', 0.65).value
        self.nms_threshold = self.declare_parameter('nms_threshold', 0.45).value
        
        # Statistics
        self.detection_count = 0
        self.frame_count = 0
        self.last_stats_time = time.time()
        
        self.get_logger().info(f"✓ TABLE DETECTOR ready!")
        self.get_logger().info(f"  - Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"  - NMS threshold: {self.nms_threshold}")
        self.get_logger().info(f"  - Detectable classes: {len(self.COCO_CLASSES)} COCO classes")
        self.get_logger().info("=" * 60)
    
    def load_detector(self):
        """Load YOLOv8 Extra Large for maximum accuracy"""
        try:
            from ultralytics import YOLO
            
            model_path = os.path.expanduser('~/.franka_vision/yolov8x.pt')
            
            if not os.path.exists(model_path):
                self.get_logger().info("Downloading YOLOv8 Extra Large model...")
                os.makedirs(os.path.dirname(model_path), exist_ok=True)
                self.detector = YOLO('yolov8x.pt')
                self.detector.save(model_path)
                self.get_logger().info(f"✓ Model saved to {model_path}")
            else:
                self.detector = YOLO(model_path)
            
            # Use CPU (can switch to GPU if available)
            self.detector.to('cpu')
            self.get_logger().info("✓ YOLOv8 Extra Large loaded (CPU inference)")
            
        except ImportError:
            self.get_logger().error("ultralytics not installed")
            self.detector = None
        except Exception as e:
            self.get_logger().error(f"Failed to load detector: {e}")
            self.detector = None
    
    def camera_info_callback(self, msg: CameraInfo):
        """Load camera intrinsics"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.frame_id = msg.header.frame_id
            self.get_logger().info(f"✓ Camera calibrated: {self.frame_id}")
    
    def image_callback(self, msg: Image):
        """Process image and detect objects"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]
            
            self.frame_count += 1
            
            # Run inference
            results = self.detector(frame, conf=self.confidence_threshold, verbose=False)
            
            # Parse detections
            detections = []
            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    class_name = self.COCO_CLASSES[cls_id] if cls_id < len(self.COCO_CLASSES) else f"class_{cls_id}"
                    
                    detections.append({
                        'class': class_name,
                        'class_id': cls_id,
                        'confidence': conf,
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'center': ((int(x1) + int(x2)) // 2, (int(y1) + int(y2)) // 2)
                    })
            
            # Publish detections
            if detections:
                self.detection_count += len(detections)
                self.publish_markers(detections, height, width)
                self.publish_annotated_image(frame, detections)
            
            # Log stats periodically
            elapsed = time.time() - self.last_stats_time
            if elapsed > 5.0:
                fps = self.frame_count / elapsed
                self.get_logger().info(
                    f"[{self.frame_count:05d}] FPS: {fps:.1f} | "
                    f"Total detections: {self.detection_count} | "
                    f"This frame: {len(detections)} objects"
                )
                self.frame_count = 0
                self.last_stats_time = time.time()
        
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def publish_markers(self, detections: List[Dict], height: int, width: int):
        """Publish detections as RViz markers"""
        marker_array = MarkerArray()
        
        for i, det in enumerate(detections):
            marker = Marker()
            marker.header.frame_id = self.frame_id or 'ee_d435i_color_optical_frame'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Bounding box center (normalized to depth space)
            cx, cy = det['center']
            marker.pose.position.x = cx / width * 0.6  # Rough scaling
            marker.pose.position.y = cy / height * 0.45
            marker.pose.position.z = 0.5
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color by class
            color_idx = det['class_id'] % len(self.COLORS)
            color = self.COLORS[color_idx]
            marker.color.r = color[2] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[0] / 255.0
            marker.color.a = 0.8
            
            # Text label
            marker.text = f"{det['class']}\n{det['confidence']:.2f}"
            
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)
    
    def publish_annotated_image(self, frame: np.ndarray, detections: List[Dict]):
        """Draw and publish annotated image with clear labels"""
        annotated = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color_idx = det['class_id'] % len(self.COLORS)
            color = self.COLORS[color_idx]
            
            # Draw thick bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)
            
            # Draw label with better visibility
            label = f"{det['class']} {det['confidence']:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            thickness = 2
            
            # Get text size
            text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
            
            # Position label above box with padding
            label_y = max(y1 - 10, 25)
            label_x = x1
            
            # Draw filled background rectangle for label
            cv2.rectangle(
                annotated,
                (label_x - 5, label_y - text_size[1] - 8),
                (label_x + text_size[0] + 5, label_y + 5),
                color,
                -1
            )
            
            # Draw white text on colored background
            cv2.putText(
                annotated,
                label,
                (label_x, label_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness
            )
        
        # Publish annotated image
        msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.annotated_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TableDetector()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
