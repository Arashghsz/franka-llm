"""
Multi-camera launch file for Franka LLM
- Fixed D415 for workspace view
- D435i at end-effector
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_cameras')
    
    # Launch fixed camera
    fixed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fixed_camera.launch.py')
        )
    )
    
    # Launch end-effector camera
    ee_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'ee_camera.launch.py')
        )
    )
    
    return LaunchDescription([
        fixed_camera_launch,
        ee_camera_launch,
    ])
