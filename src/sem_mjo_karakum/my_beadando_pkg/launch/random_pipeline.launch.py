from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_beadando_pkg',
            executable='random_sensor',
            name='random_sensor',
            parameters=[{'period': 0.5}],
            output='screen'
        ),
        Node(
            package='my_beadando_pkg',
            executable='random_processor',
            name='random_processor',
            output='screen'
        ),
    ])
