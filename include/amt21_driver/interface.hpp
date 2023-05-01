/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_ENCODER_AMT21_INTERFACE_H
#define OPENVMP_ENCODER_AMT21_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_encoder/implementation.hpp"
#include "remote_serial_bus/interface.hpp"
#include "std_msgs/msg/float32.hpp"

namespace amt21_driver {

class Interface final : public remote_encoder::Implementation {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

  rclcpp::Parameter param_model;
  rclcpp::Parameter param_addr;

  virtual bool has_position() override { return true; }

 protected:
  bool variant_14bit_;
  bool variant_multi_turn_;
  bool variant_adj_rate_;

  virtual void position_get_real_() override;
  // AMT21* does not support reading the velocity.
  // virtual double get_current_velocity_() override;

 private:
  std::shared_ptr<remote_serial_bus::Interface> prov_;
  uint8_t addr_;

  int read_2_bytes(uint8_t addr, uint16_t &result);
};

}  // namespace amt21_driver

#endif  // OPENVMP_ENCODER_AMT21_INTERFACE_H
