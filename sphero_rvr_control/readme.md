# sphero_rvr_control

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](http://wiki.ros.org/noetic/Installation/Ubuntu)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The control package contains launch and configuration files typical of a ros2_control package. The control is implemented using `sphero_rvr_control` and `sphero_rvr_controllers`.

## run hardware interface

You can start the base control layer (robot_state_publisher + ros2_control_node) from this package.

- Real hardware (default):

```bash
ros2 launch sphero_rvr_control sphero_rvr_control.launch
```

- Simulated (no hardware required):

Set the `simulated` argument to `true` in the `control.yaml` config file to run in hardware-less mode.
In simulated mode, the hardware interface ignores writes and computes state by time-integrating the commanded
wheel velocities, allowing you to test upper layers (controllers, teleop, navigation) without the robot attached.

## interfaces

The `sphero_rvr_control` package provides a `SpheroRvrHardwareInterface` class that implements the `ros2_control` hardware interface for the Sphero RVR robot. This class handles communication with the robot's hardware, including reading sensor data and sending commands to the motors and LEDs.

The hardware interface supports the following features:

- Differential drive control for skid-steered movement
- Managed LED indicators:
  - Brakelight LEDs (red) on the rear
  - Undercarriage LEDs (white)
- LED control for the headlight and status LEDs via ros services
  - Headlight LED control service: `~/set_headlight`
  - Status LED control service: `~/set_status_led`
- Sensor data reading for odometry, IMU, ambient light, and color sensors
- Simulated mode for testing without hardware
- Battery level indicator via LED on battery door
