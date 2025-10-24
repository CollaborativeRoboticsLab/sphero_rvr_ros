# Controllers

The `controllers` package is responsible for managing the robot's controllers. This may include PID controllers, trajectory controllers, and other types of control systems.

## Controllers

The sphero rvr robot is skid steered. Therefore, a `differential_drive_controller` is used to manage the movement of the robot. This controller takes in velocity commands and translates them into individual wheel speeds for the left and right wheels.

The drive command can come from various sources, such as teleoperation nodes, autonomous navigation stacks, or custom control algorithms.

The controllers are multiplexed using the `twist_mux` package.

## launch controllers

Teleop and joy control are also provided by controllers. To launch the controllers, use the following command:

```bash
ros2 launch sphero_rvr_controllers controllers.launch
```
