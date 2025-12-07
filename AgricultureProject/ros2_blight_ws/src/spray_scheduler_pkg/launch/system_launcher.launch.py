from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    

    delayed_nodes = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='spray_scheduler_pkg',
                executable='spray_scheduler',
                name='spray_scheduler',
                output='screen'
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
        ]
    )

    return LaunchDescription([
        delayed_nodes
    ])

