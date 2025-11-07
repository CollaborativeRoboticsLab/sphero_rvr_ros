# rvr_autokit_driver

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS 2 driver for the Sparkfun Autonomous Kit for Sphero RVR, including GPS and distance sensor support.

## UNTESTED WARNING

## Features

### Sparkfun AutoKit Driver

- Publishes GPS data (NavSatFix) from Qwiic Titan GPS module
- Publishes distance data (LaserScan) from VL53L1X time-of-flight sensor
- Publishes GPS time reference

### MQTT Fleet Logger

- Publishes RVR telemetry (GPS position, battery) to MQTT broker
- Configurable robot ID and broker settings
- JSON-formatted messages for fleet monitoring

## Hardware Requirements

- Sparkfun Autonomous Kit for Sphero RVR
  - Qwiic Titan GPS module
  - VL53L1X distance sensor

## Runtime Dependencies

Install Python hardware libraries:

```bash
sudo apt-get update && sudo apt-get install -y python3-pip
python3 -m pip install --user paho-mqtt sparkfun-qwiic-vl53l1x sparkfun-qwiic-titan-gps pynmea2
```

## Usage

### Launch AutoKit Driver

```bash
ros2 launch rvr_autokit_driver rvr_autokit.launch.py
```

### Launch Fleet Logger

```bash
ros2 launch rvr_autokit_driver fleet_logger.launch.py
```

## Parameters

### sparkfun_autokit_driver

- `loop_rate` (float, default: 4.0): Sensor polling rate in Hz

### mqtt_fleet_logger

- `robot_id` (string, default: 'rvr_01'): Unique robot identifier
- `mqtt_broker` (string, default: 'localhost'): MQTT broker hostname/IP
- `mqtt_port` (int, default: 1883): MQTT broker port
- `mqtt_topic_prefix` (string, default: 'fleet/rvr'): MQTT topic prefix
- `publish_rate` (float, default: 1.0): Telemetry publish rate in Hz

## Topics

### sparkfun_autokit_driver

**Publications:**

- `scan` (sensor_msgs/LaserScan): Distance sensor reading as single-point scan
- `gps` (sensor_msgs/NavSatFix): GPS position fix
- `time_ref` (sensor_msgs/TimeReference): GPS time reference

### mqtt_fleet_logger

**Subscriptions:**

- `/rvr_autokit/gps` (sensor_msgs/NavSatFix): GPS position data
- `/rvr_driver/battery` (std_msgs/Float32): Battery percentage

**MQTT Publishing:**

Topic: `{mqtt_topic_prefix}/{robot_id}/telemetry`

Payload format (JSON):

```json
{
  "robot_id": "rvr_01",
  "timestamp": 1698765432,
  "gps": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 10.5
  },
  "battery": 85.5
}
```
