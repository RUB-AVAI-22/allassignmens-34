from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_task',
            namespace='camera_node',
            executable='camera_task_node',
            name='camera_node'
        ),
        Node(
            package='camera_task',
            namespace='image_processing_node',
            executable='camera_task_node',
            name='image_processing_node'
        ),
        Node(
            package='camera_task',
            namespace='image_display_node',
            executable='camera_task_node',
            name='image_display_node'
        )
    ])