# ROS2 driver for CUI Devices AMT21

[![License](./apache20.svg)](./LICENSE.txt)

This packages supports the entire AMT21 product series using RS485.

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
