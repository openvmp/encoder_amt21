/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "amt21_driver/node.hpp"

namespace amt21_driver {

Node::Node() : rclcpp::Node::Node("encoder_amt21") {
  intf_ = std::make_shared<Interface>(this);
}

}  // namespace amt21_driver
