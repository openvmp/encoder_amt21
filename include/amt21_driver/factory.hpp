/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-18
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_ENCODER_AMT21_FACTORY_H
#define OPENVMP_REMOTE_ENCODER_AMT21_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/interface.hpp"

namespace amt21_driver {

class Factory {
 public:
  static std::shared_ptr<remote_encoder::Interface> New(rclcpp::Node *node);
};

}  // namespace amt21_driver

#endif  // OPENVMP_REMOTE_ENCODER_AMT21_FACTORY_H