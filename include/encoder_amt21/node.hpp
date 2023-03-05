/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_AMT21_NODE_H
#define OPENVMP_ENCODER_AMT21_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "encoder_amt21/interface.hpp"

namespace encoder_amt21 {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<Interface> intf_;
};

}  // namespace encoder_amt21

#endif  // OPENVMP_ENCODER_AMT21_NODE_H
