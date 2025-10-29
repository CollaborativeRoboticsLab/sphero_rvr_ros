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
