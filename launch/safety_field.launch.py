from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_field',
            executable='lidar_safety_node',
            name='lidar_safety',
            parameters=['config/safety_params.yaml']
        )
    ])