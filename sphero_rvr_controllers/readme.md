# sphero rvr controllers

The `controllers` package is responsible for managing the robot's controllers. This may include PID controllers, trajectory controllers, state and sensor broadcasters, and other types of controllers.

## controllers

The sphero rvr robot is skid steered. Therefore, a `differential_drive_controller` is used to manage the movement of the robot. This controller takes in velocity commands and translates them into individual wheel speeds for the left and right wheels.

The drive command can come from various sources, such as teleoperation nodes, autonomous navigation stacks, or custom control algorithms.

The controllers are multiplexed using the `twist_mux` package.

## Sensor Broadcasters

This package also provides custom sensor broadcasters for the RVR's onboard sensors:

### 1. Ambient Light Sensor Broadcaster

- **Type:** `sphero_rvr_controllers/AmbientLightSensorBroadcaster`
- **Publishes:** `~/ambient_light` (`sensor_msgs/Illuminance`)
- **Purpose:** Broadcasts ambient light intensity in lux

### 2. Color Sensor Broadcaster

- **Type:** `sphero_rvr_controllers/ColorSensorBroadcaster`
- **Publishes:** `~/color` (`std_msgs/ColorRGBA`)
- **Purpose:** Broadcasts RGBC color sensor values (normalized to 0-1)

## Launch Controllers

Teleop and joy control are also provided by controllers. To launch the controllers, use the following command:

```bash
ros2 launch sphero_rvr_controllers sphero_rvr_controllers.launch
```
