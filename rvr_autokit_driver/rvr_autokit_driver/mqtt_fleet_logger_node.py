#!/usr/bin/env python3

"""
MQTT Fleet Logger for RVR

Publishes RVR telemetry to an MQTT broker for fleet monitoring.
Note: This is a placeholder implementation. Configure MQTT broker details via parameters.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import json

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


class MQTTFleetLoggerNode(Node):
    def __init__(self):
        super().__init__('mqtt_fleet_logger')

        if not MQTT_AVAILABLE:
            self.get_logger().error('paho-mqtt not installed. Install with: pip install paho-mqtt')
            return

        # Parameters
        self.robot_id = self.declare_parameter('robot_id', 'rvr-001').value
        self.mqtt_broker = self.declare_parameter(
            'mqtt_broker', 'localhost').value
        self.mqtt_port = self.declare_parameter('mqtt_port', 1883).value
        self.mqtt_topic_prefix = self.declare_parameter(
            'mqtt_topic_prefix', 'fleet/rvr').value
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 1.0).value)

        # MQTT client
        self.mqtt_client = mqtt.Client(client_id=f'rvr_logger_{self.robot_id}')
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        # Data cache
        self.gps_data = None
        self.battery_data = None

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, '/autokit/gps/fix', self.gps_callback, 10)
        self.battery_sub = self.create_subscription(
            Float32, '/rvr_driver/battery', self.battery_callback, 10)

        # Timer to publish to MQTT
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_to_mqtt)

        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f'Disconnected from MQTT broker with code {rc}')

    def gps_callback(self, msg):
        self.gps_data = msg

    def battery_callback(self, msg):
        self.battery_data = msg.data

    def publish_to_mqtt(self):
        """Publish telemetry to MQTT"""
        if not MQTT_AVAILABLE:
            return

        payload = {
            'robot_id': self.robot_id,
            'timestamp': self.get_clock().now().to_msg().sec,
        }

        if self.gps_data:
            payload['gps'] = {
                'latitude': self.gps_data.latitude,
                'longitude': self.gps_data.longitude,
                'altitude': self.gps_data.altitude,
            }

        if self.battery_data is not None:
            payload['battery'] = self.battery_data

        topic = f'{self.mqtt_topic_prefix}/{self.robot_id}/telemetry'
        try:
            self.mqtt_client.publish(topic, json.dumps(payload))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish to MQTT: {e}')

    def __del__(self):
        if MQTT_AVAILABLE and hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTFleetLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
