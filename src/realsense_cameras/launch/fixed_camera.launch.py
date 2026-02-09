"""
Fixed D415 camera launch file (workspace view)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fixed_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='fixed_camera',
        namespace='cameras/fixed',
        parameters=[{
            'device_type': 'D415',
            'serial_no': '821312061433',
            'camera_name': 'fixed_d415',
            'rgb_camera.profile': '640x480x30',
            'enable_color': True,
            'depth_module.profile': '640x480x30',
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_gyro': False,
            'enable_accel': False,
            'temporal_filter.enable': True,
            'publish_tf': True,
            'tf_publish_rate': 0.0,
        }],
        output='screen',
    )
    
    return LaunchDescription([fixed_camera])
