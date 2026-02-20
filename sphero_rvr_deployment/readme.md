# Sphero RVR Deployment

Automatically run application and avoid version conflicts when deploying Sphero RVR ROS2 packages.

## environment

create and edit the `.env` file to set environment variables for the deployment.

```bash
# Copy the example environment file
cp .env.example .env

# Edit the .env file to set your desired configuration
```

### environment variables

| Variable Name | Description | Default Value |
|---------------|-------------|---------------|
| ROBOT_ID | Unique identifier for the Sphero RVR robot. | `rvr-001` |
| ROS_DOMAIN_ID | Set the ROS2 domain ID for communication isolation. | `0` |
| RMW_IMPLEMENTATION | Specify the ROS2 RMW implementation to use (e.g., `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`). | `rmw_fastrtps_cpp` |
| ROS_LOCALHOST_ONLY | Set to `0` to allow ROS2 communication over the network, or `1` to restrict to localhost. | `0` |
| ROS_AUTOMATIC_DISCOVERY_RANGE | Set the range for automatic discovery of ROS2 nodes on network. | `SUBNET` |
| DISCOVERY_SERVER_IP | IP address of the ROS2 discovery server for multi-robot communication. | `127.0.0.1` |
| DISCOVERY_SERVER_PORT | Port of the ROS2 discovery server for multi-robot communication. | `11811` |
| ROS_SUPER_CLIENT | Set to `True` to enable ROS2 super client mode for proper ros2 cli functionality. | `True` |
| MQTT_BROKER | URL of the MQTT broker for fleet messages. | `localhost` |
| MQTT_PORT | Port of the MQTT broker. | `1883` |
| MQTT_TOPIC_PREFIX | Topic prefix for MQTT messages. | `fleet/rvr` |
| RVR_DRIVER_SOURCE | Git repository URL for the Sphero RVR driver source code. | `https://github.com/collaborativeroboticslab/sphero_rvr_driver_py.git` |
| SERIAL_PORT | Serial port for Sphero RVR connection. | `/dev/serial0` |
| I2C_BUS | I2C bus for Sphero RVR connection. | `/dev/i2c-1` |

## run - docker-compose

This command will start the deployment environment in detached mode. It sets up the necessary Docker containers and services to run the Sphero RVR ROS2 application.

```bash
# Start the deployment environment
docker-compose up -d
```

## run - robot_upstart

To ensure that the Sphero RVR deployment starts automatically on system boot, you can set up a systemd service. The robot_upstart package provides a convenient way to manage this.

```bash
# build the rvr ros package
colcon build

# source the install setup file
source install/setup.bash

# use robot_upstart to install the systemd service
ros2 run robot_upstart install --package sphero_rvr_bringup --launch-file sphero_rvr_bringup.launch
```
