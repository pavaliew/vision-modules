from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Set memory optimization environment variables
    os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'
    
    return LaunchDescription([
        # Start camera publisher first with lower resolution
        Node(
            package='ai_package',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{
                'device': 0,
                'frame_width': 640,    # Reduced resolution
                'frame_height': 480,   # Reduced resolution  
                'fps': 15,             # Lower FPS
                'topic_name': '/camera/image_raw'
            }],
            output='screen'
        ),
        
        # Start landing node after a short delay with memory optimization
        TimerAction(
            period=3.0,  # Wait 3 seconds for camera to initialize
            actions=[
                Node(
                    package='ai_package',
                    executable='landing_takeoff_node',
                    name='landing_takeoff_node',
                    parameters=[{
                        'model_path': 'yolov10n.pt',  # Use nano model
                        'imgsz': 320,               # Small input size
                        'conf_threshold': 0.5,
                        'max_queue_size': 1         # Process one frame at a time
                    }],
                    output='screen'
                )
            ]
        )
    ])
