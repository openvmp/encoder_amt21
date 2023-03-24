# OpenVMP

[![License](./apache20.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## ROS2 package for CUI Devices AMT21

This packages interacts with  encoders using RS485.
It is expected to support the entire AMT21 product series.

## Ready for ros2\_control

This package works with
[ros2\_control](https://github.com/ros-controls/ros2_control) via
[remote\_hardware\_interface](https://github.com/openvmp/remote_hardware_interface).

### Generic encoder interface

This package implements [the generic encoder interface](https://github.com/openvmp/encoder/).

```shell
$ ros2 service list
...
/openvmp/robot_EE3U/encoder/front_body_joint/get_position
/openvmp/robot_EE3U/encoder/front_body_joint/get_velocity
...
```
