/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "amt21_driver/factory.hpp"

#include <exception>

#include "amt21_driver/interface.hpp"
#include "remote_encoder/interface_remote.hpp"

namespace amt21_driver {

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

}  // namespace amt21_driver
