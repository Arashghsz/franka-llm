#!/usr/bin/env python3
"""
VLM Node - Jetson AGX Orin
Subscribes to camera image from PC controller via ROS 2
Sends image to LLaVA 7B via Ollama for scene analysis
Publishes explanation of table contents to /vlm/explanation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
import requests
import base64
import json
import time
from pathlib import Path


class VLMNode(Node):
    """VLM Node for semantic understanding of table scene"""

    def __init__(self):
        super().__init__('vlm_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/cameras/ee/ee_camera/color/image_raw')  # EE camera by default
        self.declare_parameter('use_compressed', True)  # Use compressed images by default for network efficiency
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('vlm_model', 'llava:7b')
        self.declare_parameter('auto_analyze', False)  # Disable automatic scene analysis
        self.declare_parameter('analysis_rate', 1.0)  # Hz (only used if auto_analyze=True)
        self.declare_parameter('timeout', 30)  # seconds
        self.declare_parameter('debug', False)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.ollama_host = self.get_parameter('ollama_host').value
        self.vlm_model = self.get_parameter('vlm_model').value
        self.auto_analyze = self.get_parameter('auto_analyze').value
        self.analysis_rate = self.get_parameter('analysis_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Depth resolution data
        self.latest_depth_image = None
        self.camera_intrinsics = None
        
        # Subscribe to camera (compressed or raw)
        if self.use_compressed:
            # Subscribe to compressed images (better for network transmission)
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.camera_topic + '/compressed',
                self.compressed_image_callback,
                10
            )
        else:
            # Subscribe to raw images
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                10
            )
        
        # Publish analysis results
        self.analysis_pub = self.create_publisher(
            String,
            '/vlm/explanation',
            10
        )
        
        # Status publisher for debugging
        self.status_pub = self.create_publisher(
            String,
            '/vlm/status',
            10
        )
        
        # Subscribe to grounding requests from LLM/coordinator
        self.grounding_request_sub = self.create_subscription(
            String,
            '/vlm_request',
            self.grounding_request_callback,
            10
        )
        
        # Subscribe to depth image for 3D position resolution
        self.depth_sub = self.create_subscription(
            Image,
            '/cameras/ee/ee_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        # Subscribe to camera info for intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/cameras/ee/ee_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publisher for 3D position (center of scene)
        self.position_pub = self.create_publisher(
            PoseStamped,
            '/vlm_center_position',
            10
        )
        
        # Rate limiting
        self.min_interval = 1.0 / self.analysis_rate
        self.last_analysis_time = time.time()
        self.latest_image = None
        self.image_lock = False
        
        # Check Ollama connection
        self.check_ollama_health()
        
        self.get_logger().info(
            f'VLM Node initialized:\n'
            f'  Camera topic: {self.camera_topic}\n'
            f'  Use compressed: {self.use_compressed}\n'
            f'  Ollama host: {self.ollama_host}\n'
            f'  VLM model: {self.vlm_model}\n'
            f'  Analysis rate: {self.analysis_rate} Hz\n'
            f'  Debug mode: {self.debug}'
        )
    
    def check_ollama_health(self):
        """Check if Ollama is running and model is available"""
        try:
            response = requests.get(
                f'{self.ollama_host}/api/tags',
                timeout=5
            )
            if response.status_code == 200:
                models = response.json().get('models', [])
                model_names = [m.get('name', '') for m in models]
                
                if self.vlm_model in model_names or any(self.vlm_model in m for m in model_names):
                    self.get_logger().info(f'✓ Ollama is running, {self.vlm_model} is available')
                    return True
                else:
                    self.get_logger().warn(
                        f'⚠ Ollama is running but {self.vlm_model} not found. Available models: {model_names}\n'
                        f'Pull the model with: ollama pull {self.vlm_model}'
                    )
                    return False
            else:
                self.get_logger().error(f'Ollama API error: {response.status_code}')
                return False
        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                f'✗ Cannot connect to Ollama at {self.ollama_host}\n'
                f'Start Ollama with: ollama serve'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Error checking Ollama: {str(e)}')
            return False
    
    def image_callback(self, msg: Image):
        """Receive and store latest image"""
        try:
            # Store latest image for processing
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Only auto-analyze if enabled
            if self.auto_analyze:
                current_time = time.time()
                if current_time - self.last_analysis_time >= self.min_interval:
                    self.process_image()
                    self.last_analysis_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def compressed_image_callback(self, msg: CompressedImage):
        """Receive and decompress latest compressed image"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Only auto-analyze if enabled
            if self.auto_analyze:
                current_time = time.time()
                if current_time - self.last_analysis_time >= self.min_interval:
                    self.process_image()
                    self.last_analysis_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error in compressed image callback: {str(e)}')
    
    def process_image(self):
        """Process the latest image with VLM"""
        if self.latest_image is None:
            return
        
        try:
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Send to VLM
            self.get_logger().info('Analyzing table scene with VLM...')
            explanation = self.query_vlm(image_base64)
            
            if explanation:
                # Get 3D position of center pixel
                center_3d = self.get_center_pixel_3d_position()
                
                # Publish explanation
                msg = String()
                msg.data = explanation
                self.analysis_pub.publish(msg)
                
                self.get_logger().info(f'Published explanation: {explanation[:80]}...')
                
                # Publish 3D position if available
                if center_3d:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'ee_d435i_color_optical_frame'
                    pose_msg.pose.position.x = center_3d['position'][0]
                    pose_msg.pose.position.y = center_3d['position'][1]
                    pose_msg.pose.position.z = center_3d['position'][2]
                    pose_msg.pose.orientation.w = 1.0
                    
                    self.position_pub.publish(pose_msg)
                    
                    self.get_logger().info(
                        f'Center pixel 3D position: '
                        f'X={center_3d["position"][0]:.3f}, '
                        f'Y={center_3d["position"][1]:.3f}, '
                        f'Z={center_3d["position"][2]:.3f}'
                    )
                
                # Publish status
                status_msg = String()
                status_msg.data = f'ANALYSIS_COMPLETE: {time.strftime("%H:%M:%S")}'
                self.status_pub.publish(status_msg)
            else:
                status_msg = String()
                status_msg.data = f'ANALYSIS_FAILED: Could not get VLM response'
                self.status_pub.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            status_msg = String()
            status_msg.data = f'ERROR: {str(e)}'
            self.status_pub.publish(status_msg)
    
    def grounding_request_callback(self, msg: String):
        """
        Handle analysis requests from LLM coordinator
        Expected format: 
        - Scene description: {"type": "scene_description"}
        - Locate object: {"type": "locate", "object": "red cup"}
        """
        try:
            request = json.loads(msg.data)
            request_type = request.get('type', 'scene_description')
            target_object = request.get('object', None)
            
            if self.latest_image is None:
                self.get_logger().warn('Analysis requested but no image available')
                return
            
            self.get_logger().info(f'{request_type.upper()} requested')
            
            if request_type == 'locate' and target_object:
                # Find specific object and get its center
                self.process_object_localization(target_object)
            else:
                # General scene description with image center depth
                self.process_image()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in request: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in request callback: {str(e)}')
    
    def depth_callback(self, msg: Image):
        """Store latest depth image for 3D position resolution"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert from mm to meters if needed
            if self.latest_depth_image.dtype == np.uint16:
                self.latest_depth_image = self.latest_depth_image.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics for deprojection"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(f'Camera intrinsics loaded: {self.camera_intrinsics}')
    
    def get_center_pixel_3d_position(self) -> dict:
        """
        Get 3D position of center pixel from depth camera
        
        Returns:
            dict with 'position' [x, y, z] in meters, or None if unavailable
        """
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            return None
        
        try:
            height, width = self.latest_depth_image.shape
            center_x = width // 2
            center_y = height // 2
            
            # Get depth at center pixel
            depth_z = float(self.latest_depth_image[center_y, center_x])
            
            if depth_z <= 0:
                self.get_logger().warn(f'Invalid depth at center pixel: {depth_z}')
                return None
            
            # Deproject to 3D
            fx = self.camera_intrinsics['fx']
            fy = self.camera_intrinsics['fy']
            ppx = self.camera_intrinsics['ppx']
            ppy = self.camera_intrinsics['ppy']
            
            x = (center_x - ppx) * depth_z / fx
            y = (center_y - ppy) * depth_z / fy
            z = depth_z
            
            return {
                'position': [x, y, z],
                'pixel': [center_x, center_y],
                'depth': depth_z
            }
        except Exception as e:
            self.get_logger().error(f'Error getting center pixel 3D position: {e}')
            return None
    
    def query_vlm_for_object_center(self, image_base64: str, target_object: str) -> dict:
        """
        Query VLM to locate object and return its center pixel
        
        Args:
            image_base64: Base64 encoded image
            target_object: Name of object to locate (e.g., "red cup")
            
        Returns:
            {
                "center": [x, y],  # pixel coordinates of object center
                "description": "The red cup is on the left side",
                "confidence": "high"
            }
        """
        try:
            prompt = (
                f'I need to locate the "{target_object}" in this image. '
                f'Please identify where it is and provide the CENTER PIXEL coordinates.\n\n'
                f'Respond in JSON format:\n'
                f'{{\n'
                f'  "center": [x, y],\n'
                f'  "description": "brief description of where the object is",\n'
                f'  "confidence": "high|medium|low"\n'
                f'}}\n\n'
                f'The center should be the approximate pixel coordinates at the middle of the object.\n'
                f'If you cannot find the object, set center to null.'
            )
            
            payload = {
                'model': self.vlm_model,
                'prompt': prompt,
                'images': [image_base64],
                'stream': False,
                'temperature': 0.3,
                'format': 'json',
            }
            
            response = requests.post(
                f'{self.ollama_host}/api/generate',
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                vlm_response = result.get('response', '{}')
                
                try:
                    data = json.loads(vlm_response)
                    if data.get('center') and len(data['center']) == 2:
                        return data
                    else:
                        self.get_logger().warn(f'VLM could not locate {target_object}')
                        return None
                except json.JSONDecodeError:
                    self.get_logger().error(f'VLM returned invalid JSON: {vlm_response}')
                    return None
            else:
                self.get_logger().error(f'VLM API error: {response.status_code}')
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'VLM timeout after {self.timeout}s')
            return None
        except Exception as e:
            self.get_logger().error(f'Error querying VLM for object center: {str(e)}')
            return None
    
    def query_vlm(self, image_base64: str) -> str:
        """
        Query LLaVA 7B with the image
        Prompts the model to describe table contents
        """
        try:
            prompt = (
                'Analyze this table scene and describe what objects are on the table. '
                'Be specific about:\n'
                '1. What objects are present\n'
                '2. Their approximate locations (left, center, right, front, back)\n'
                '3. Their colors and sizes\n'
                '4. The overall arrangement of items\n'
                'Be concise but informative.'
            )
            
            payload = {
                'model': self.vlm_model,
                'prompt': prompt,
                'images': [image_base64],
                'stream': False,
                'temperature': 0.5,  # Lower temperature for more focused responses
            }
            
            if self.debug:
                self.get_logger().info(f'Sending request to {self.ollama_host}/api/generate')
            
            response = requests.post(
                f'{self.ollama_host}/api/generate',
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                explanation = result.get('response', 'No response from VLM')
                return explanation.strip()
            else:
                self.get_logger().error(
                    f'VLM API error: {response.status_code}\n'
                    f'Response: {response.text}'
                )
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'VLM timeout after {self.timeout}s')
            return None
        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                f'Cannot connect to Ollama at {self.ollama_host}\n'
                f'Make sure Ollama is running: ollama serve'
            )
            return None
        except Exception as e:
            self.get_logger().error(f'Error querying VLM: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
