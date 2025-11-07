from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rvr_autokit_driver',
            executable='mqtt_fleet_logger',
            name='rvr_fleet_logger',
            parameters=[{
                'robot_id': 'rvr_01',
                'mqtt_broker': 'localhost',
                'mqtt_port': 1883,
                'mqtt_topic_prefix': 'fleet/rvr',
                'publish_rate': 1.0,
            }]
        )
    ])
