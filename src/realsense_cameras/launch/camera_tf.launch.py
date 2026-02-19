"""
Camera TF Publisher - Static transform from robot hand to camera
Based on calibration from friend's ee_camera_tf.launch

This publishes the static transform: panda_hand -> camera_color_optical_frame
Calibration values: xyz="0.0458961 -0.0368559 0.0567165" (quaternion provided)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch static TF broadcaster for end-effector camera.
    
    Transform: fr3_hand -> ee_d435i_color_optical_frame
    Position (m): x=0.0459, y=-0.0369, z=0.0567
    Orientation (quaternion): x=-0.000401, y=-0.00425, z=0.698, w=0.716
    
    NOTE: This connects the camera to robot hand.
    Make sure robot is running and publishing fr3_hand frame!
    """
    
    # Static TF: fr3_hand -> ee_d435i_color_optical_frame
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            # Translation (x, y, z in meters)
            '0.0458961', '-0.0368559', '0.0567165',
            # Rotation (quaternion: x, y, z, w)
            '-0.000400857', '-0.00425145', '0.698275', '0.715817',
            # Parent frame (FR3 robot hand)
            'fr3_hand',
            # Child frame (D435i camera optical frame)
            'ee_d435i_color_optical_frame'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_tf_publisher
    ])
