import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch both the autokit driver and fleet logger together
        Node(
            package='rvr_autokit_driver',
            executable='sparkfun_autokit_driver',
            name='rvr_autokit',
            parameters=[{
                'loop_rate': 4.0,
            }]
        ),
        Node(
            package='rvr_autokit_driver',
            executable='mqtt_fleet_logger',
            name='rvr_fleet_logger',
            parameters=[{
                # get these from environment variables or set defaults
                'robot_id': os.environ.get('ROBOT_ID', 'rvr-001'),
                'mqtt_broker': os.environ.get('MQTT_BROKER', 'localhost'),
                'mqtt_port': int(os.environ.get('MQTT_PORT', 1883)),
                'mqtt_topic_prefix': os.environ.get('MQTT_TOPIC_PREFIX', 'fleet/rvr'),
                'publish_rate': 1.0,
            }]
        )
    ])
