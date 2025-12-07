from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spray_scheduler_pkg',
            executable='spray_scheduler',
            name='spray_scheduler'
        ),
        Node(
            package='spray_decider',
            executable='spray_decider',
            name='spray_decider',
            output='screen'
        ),
        Node(
            package='weather_reporter',
            executable='weather_node',
            name='weather_reporter',
            output='screen'
        ),
        Node(
            package='blight_detector',
            executable='blight_detector',
            name='blight_detector',
            output='screen'
        ),
    ])

