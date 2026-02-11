#!/usr/bin/env python3
"""
VLM Node - Jetson AGX Orin
Subscribes to camera image from PC controller via ROS 2
Sends image to LLaVA 7B via Ollama for scene analysis
Publishes explanation of table contents to /vlm/explanation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
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
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('use_compressed', True)  # Use compressed images by default for network efficiency
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('vlm_model', 'llava:7b')
        self.declare_parameter('analysis_rate', 1.0)  # Hz
        self.declare_parameter('timeout', 30)  # seconds
        self.declare_parameter('debug', False)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.ollama_host = self.get_parameter('ollama_host').value
        self.vlm_model = self.get_parameter('vlm_model').value
        self.analysis_rate = self.get_parameter('analysis_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
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
        
        # Publish grounding results (bbox + target)
        self.grounding_pub = self.create_publisher(
            String,
            '/vlm_grounding',
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
            
            # Check if enough time has passed since last analysis
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
            
            # Check if enough time has passed since last analysis
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
                # Publish explanation
                msg = String()
                msg.data = explanation
                self.analysis_pub.publish(msg)
                
                self.get_logger().info(f'Published explanation: {explanation[:80]}...')
                
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
        Handle grounding requests from LLM/coordinator
        Expected format: JSON with {"target": "red cup", "task": "pick"}
        """
        try:
            request = json.loads(msg.data)
            target = request.get('target', '')
            
            if not target or self.latest_image is None:
                self.get_logger().warn('Grounding request received but no target or no image available')
                return
            
            self.get_logger().info(f'Grounding request for target: {target}')
            
            # Encode current image
            _, buffer = cv2.imencode('.jpg', self.latest_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Query VLM for grounding (bbox + rationale)
            grounding_result = self.query_vlm_for_grounding(image_base64, target)
            
            if grounding_result:
                # Publish grounding result
                msg = String()
                msg.data = json.dumps(grounding_result)
                self.grounding_pub.publish(msg)
                
                self.get_logger().info(f'Published grounding: {grounding_result}')
            else:
                self.get_logger().error('Grounding failed')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in grounding request: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in grounding request callback: {str(e)}')
    
    def query_vlm_for_grounding(self, image_base64: str, target: str) -> dict:
        """
        Query VLM to locate target object and return bounding box
        
        Returns:
            {
                "target": "red cup",
                "bbox": [x1, y1, x2, y2],  # pixel coordinates
                "confidence": "high",
                "rationale": "The red cup is on the left side of the table"
            }
        """
        try:
            # Prompt VLM to provide bounding box coordinates
            # Note: LLaVA doesn't natively output structured bboxes, 
            # so we prompt it to describe location and parse the response
            # For production, use Florence-2, Qwen-VL, or VILA which have native grounding
            prompt = (
                f'I need to locate the "{target}" in this image. '
                f'Please provide:\n'
                f'1. A bounding box in the format [x1, y1, x2, y2] where coordinates are in pixels\n'
                f'2. Your confidence level (high/medium/low)\n'
                f'3. A brief rationale for why you selected this region\n\n'
                f'Respond in JSON format:\n'
                f'{{\n'
                f'  "bbox": [x1, y1, x2, y2],\n'
                f'  "confidence": "high",\n'
                f'  "rationale": "your explanation"\n'
                f'}}\n\n'
                f'If you cannot find the object, set bbox to null.'
            )
            
            payload = {
                'model': self.vlm_model,
                'prompt': prompt,
                'images': [image_base64],
                'stream': False,
                'temperature': 0.2,  # Low temperature for precise localization
                'format': 'json',  # Request JSON output
            }
            
            response = requests.post(
                f'{self.ollama_host}/api/generate',
                json=payload,
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                vlm_response = result.get('response', '{}')
                
                # Parse VLM's JSON response
                try:
                    grounding_data = json.loads(vlm_response)
                    
                    # Add target name to response
                    grounding_data['target'] = target
                    
                    # Validate bbox format
                    if grounding_data.get('bbox') and len(grounding_data['bbox']) == 4:
                        return grounding_data
                    else:
                        self.get_logger().warn(f'VLM could not locate {target}')
                        return None
                        
                except json.JSONDecodeError:
                    self.get_logger().error(f'VLM did not return valid JSON: {vlm_response}')
                    return None
            else:
                self.get_logger().error(f'VLM API error: {response.status_code}')
                return None
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'VLM grounding timeout after {self.timeout}s')
            return None
        except Exception as e:
            self.get_logger().error(f'Error in VLM grounding: {str(e)}')
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
