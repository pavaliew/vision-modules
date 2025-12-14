from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_package',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{
                'device': 2
            }]
        ),
    ])
