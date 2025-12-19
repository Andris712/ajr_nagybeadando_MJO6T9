from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    period = ParameterValue(LaunchConfiguration('period'), value_type=float)
    min_value = ParameterValue(LaunchConfiguration('min_value'), value_type=float)
    max_value = ParameterValue(LaunchConfiguration('max_value'), value_type=float)
    window_size = ParameterValue(LaunchConfiguration('window_size'), value_type=int)
    scale_factor = ParameterValue(LaunchConfiguration('scale_factor'), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('period', default_value='0.5'),
        DeclareLaunchArgument('min_value', default_value='0.0'),
        DeclareLaunchArgument('max_value', default_value='100.0'),
        DeclareLaunchArgument('window_size', default_value='10'),
        DeclareLaunchArgument('scale_factor', default_value='1.0'),

        Node(
            package='my_beadando_pkg',
            executable='random_sensor',
            name='random_sensor',
            parameters=[{
                'period': period,
                'min_value': min_value,
                'max_value': max_value,
            }],
            output='screen'
        ),

        Node(
            package='my_beadando_pkg',
            executable='random_processor',
            name='random_processor',
            parameters=[{
                'window_size': window_size,
                'scale_factor': scale_factor,
            }],
            output='screen'
        ),
    ])
