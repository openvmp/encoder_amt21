/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_amt21/factory.hpp"

#include <exception>

#include "remote_encoder/interface_remote.hpp"
#include "ros2_amt21/interface.hpp"

namespace ros2_amt21 {

std::shared_ptr<remote_encoder::Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter use_remote;
  if (!node->has_parameter("use_remote")) {
    node->declare_parameter("use_remote", true);
  }
  node->get_parameter("use_remote", use_remote);

  rclcpp::Parameter is_remote;
  if (!node->has_parameter("encoder_is_remote")) {
    node->declare_parameter("encoder_is_remote", use_remote.as_bool());
  }
  node->get_parameter("encoder_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<remote_encoder::RemoteInterface>(node);
  } else {
    return std::make_shared<Interface>(node);
  }
}

}  // namespace ros2_amt21
