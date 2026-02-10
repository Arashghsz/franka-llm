from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for VLM Node on Jetson
    Subscribes to camera feed from PC controller
    Analyzes table scene with LLaVA 7B
    Publishes explanations to /vlm/explanation
    """
    
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/color/image_raw',
        description='Camera image topic from PC controller'
    )
    
    ollama_host_arg = DeclareLaunchArgument(
        'ollama_host',
        default_value='http://localhost:11434',
        description='Ollama server host (localhost for Jetson, or remote PC IP)'
    )
    
    vlm_model_arg = DeclareLaunchArgument(
        'vlm_model',
        default_value='llava:7b',
        description='VLM model to use (llava:7b, llava:13b, vila, etc.)'
    )
    
    analysis_rate_arg = DeclareLaunchArgument(
        'analysis_rate',
        default_value='1.0',
        description='Analysis frequency in Hz'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='30',
        description='VLM response timeout in seconds'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Enable debug logging'
    )
    
    # Create VLM node
    vlm_node = Node(
        package='franka_vlm_agent',
        executable='vlm_node',
        name='vlm_node',
        output='screen',
        parameters=[
            {'camera_topic': LaunchConfiguration('camera_topic')},
            {'ollama_host': LaunchConfiguration('ollama_host')},
            {'vlm_model': LaunchConfiguration('vlm_model')},
            {'analysis_rate': LaunchConfiguration('analysis_rate')},
            {'timeout': LaunchConfiguration('timeout')},
            {'debug': LaunchConfiguration('debug')},
        ],
        remappings=[
            ('/camera/color/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        ollama_host_arg,
        vlm_model_arg,
        analysis_rate_arg,
        timeout_arg,
        debug_arg,
        vlm_node,
    ])
