from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rvr_autokit_driver',
            executable='sparkfun_autokit_driver',
            name='autokit',
            parameters=[{
                'loop_rate': 4.0,
            }]
        )
    ])
