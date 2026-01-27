#!/usr/bin/env python3
"""
LLM Node - Connects Ollama to ROS 2

Subscribes to: /user_command (String)
Publishes to: /llm_response (String)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json


class LLMNode(Node):
    
    def __init__(self):
        super().__init__('llm_node')
        
        # Parameters
        self.declare_parameter('model', 'llama3.1:8b')
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        
        self.model = self.get_parameter('model').value
        self.ollama_url = self.get_parameter('ollama_url').value
        
        # Publisher
        self.publisher = self.create_publisher(String, '/llm_response', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )
        
        # System prompt for robot task planning
        self.system_prompt = """You are a robot task planner for a Franka arm.
                Convert commands to JSON action plans.

                Available actions: pick, place, move, wait, inspect

                Respond with JSON only:
                {
                "understood": true,
                "tasks": [
                    {"action": "pick", "object": "...", "location": "...", "response time": "..."}
                ]
                }
                """
        
        self.get_logger().info(f'LLM Node started. Model: {self.model}')
        self.get_logger().info('Listening on: /user_command')
        self.get_logger().info('Publishing to: /llm_response')
    
    def query_ollama(self, prompt: str) -> str:
        """Send query to Ollama API."""
        try:
            response = requests.post(
                f'{self.ollama_url}/api/generate',
                json={
                    'model': self.model,
                    'prompt': f"{self.system_prompt}\n\nCommand: {prompt}",
                    'stream': False,
                    'options': {
                        'temperature': 0.3,
                        'num_predict': 500,
                    }
                },
                timeout=60
            )
            
            if response.status_code == 200:
                return response.json().get('response', '')
            else:
                self.get_logger().error(f'Ollama error: {response.status_code}')
                return ''
                
        except requests.exceptions.Timeout:
            self.get_logger().error('Ollama timeout')
            return ''
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return ''
    
    def command_callback(self, msg: String):
        """Handle incoming commands."""
        user_input = msg.data
        self.get_logger().info(f'Received: {user_input}')
        
        # Query LLM
        self.get_logger().info('Querying LLM...')
        response = self.query_ollama(user_input)
        
        if response:
            self.get_logger().info(f'LLM response: {response}')
            
            # Publish response
            out_msg = String()
            out_msg.data = response
            self.publisher.publish(out_msg)
        else:
            self.get_logger().error('No response from LLM')


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
