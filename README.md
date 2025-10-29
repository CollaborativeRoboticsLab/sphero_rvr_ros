# sphero_rvr_ros

[![ROS2 jazzy](https://img.shields.io/badge/ROS2-jazzy-blue)](http://wiki.ros.org/noetic/Installation/Ubuntu)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Open in Visual Studio Code](https://img.shields.io/badge/vscode-dev-blue)](https://open.vscode.dev/AIResearchLab/sphero_rvr_ros)

ROS2 meta package containing an implementation of a Sphero RVR robot using ROS2 via Sphero serial protocol.

## Hardware

The Sphero RVR robot is the base platform. You will need a way to connect to the RVR's serial interface. This is possible using a raspberry pi connected via the RVR's expansion port or a USB to serial adapter connected to your computer.

The navigation stack requires additional hardware such as a LIDAR to function properly. The RVR/pi can be upgraded using the sparkfun autonomous vehicle kit which includes GPS, distance sensors, and servo pi hat.

Additional printed payload tower can house this assembly and a hokuyo lidar and pi camera.

A servo based arm can also be added to the RVR for pick and place applications.

## Installation

(Optional): The rvr driver depends on the `sphero_rvr_sdk`.

## Contents

Some interesting packages in this application are:

- `sphero_rvr_description`: URDF description of the RVR robot.
- `sphero_rvr_control`: ROS2 control configuration for the RVR robot, implements sphero serial protocol.
- `sphero_rvr_controllers`: ROS2 node that provides a simple control interface for the RVR robot.
- `sphero_rvr_navigation`: Navigation stack configuration for the RVR robot.
- `sphero_rvr_bringup`: Launch files to bring up the RVR robot.
