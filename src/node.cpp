/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_amt21/node.hpp"

namespace ros2_amt21 {

Node::Node() : rclcpp::Node::Node("encoder_amt21") {
  intf_ = std::make_shared<Interface>(this);
}

}  // namespace ros2_amt21
