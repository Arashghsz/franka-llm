"""
End-effector D435i camera launch file
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ee_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='ee_camera',
        namespace='cameras/ee',
        parameters=[{
            'device_type': 'D435i',
            'serial_no': '845112071321',
            'camera_name': 'ee_d435i',
            'rgb_camera.profile': '640x480x30',
            'enable_color': True,
            'depth_module.profile': '640x480x30',
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'align_depth.enable': True,
            'enable_gyro': False,
            'enable_accel': False,
            'temporal_filter.enable': True,
            'enable_sync': True,
            'publish_tf': True,
            'tf_publish_rate': 0.0,
        }],
        output='screen',
    )
    
    return LaunchDescription([ee_camera])
