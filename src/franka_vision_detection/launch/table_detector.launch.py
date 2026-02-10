from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    detector_node = Node(
        package='franka_vision_detection',
        executable='table_detector',
        name='table_detector',
        output='screen',
        parameters=[
            {'confidence_threshold': 0.65},  # High confidence for reliable detections
            {'nms_threshold': 0.45},
        ],
    )

    return LaunchDescription([detector_node])
