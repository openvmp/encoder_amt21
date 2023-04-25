/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_amt21/interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>

#include "ros2_serial_bus/factory.hpp"

#ifndef DEBUG
#undef RCLCPP_DEBUG
#if 1
#define RCLCPP_DEBUG(...)
#else
#define RCLCPP_DEBUG RCLCPP_INFO
#define DEBUG 1
#endif
#endif

namespace ros2_amt21 {

Interface::Interface(rclcpp::Node *node)
    : remote_encoder::Implementation(node),
      variant_14bit_{false},
      variant_multi_turn_{false},
      variant_adj_rate_{false} {
  auto prefix = get_prefix_();

  prov_ = ros2_serial_bus::Factory::New(node);

  RCLCPP_DEBUG(node_->get_logger(),
               "Interface::Interface():"
               " started");

  node->declare_parameter("encoder_model", "amt212a");
  node->get_parameter("encoder_model", param_model);
  node->declare_parameter("encoder_amt21_addr", 84);
  node->get_parameter("encoder_amt21_addr", param_addr);
  addr_ = param_addr.as_int();

  std::string model = param_model.as_string();
  std::string model_normalized;
  for (auto &&ch : model) {
    if (ch != '-' && ch != '_' && ch != ' ') {
      model_normalized.push_back(std::tolower(ch));
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Encoder model: %s",
              model_normalized.c_str());

  // There is no logical differences between amt212 and amt213.
  // They only differ by the physical form factor.
  if (model_normalized.find("amt212") == 0 ||
      model_normalized.find("amt213") == 0) {
    switch (model_normalized[6]) {
      case 'a':
        variant_14bit_ = false;
        variant_multi_turn_ = false;
        variant_adj_rate_ = false;
        break;
      case 'b':
        variant_14bit_ = true;
        variant_multi_turn_ = false;
        variant_adj_rate_ = false;
        break;
      case 'c':
        variant_14bit_ = false;
        variant_multi_turn_ = true;
        variant_adj_rate_ = false;
        break;
      case 'd':
        variant_14bit_ = true;
        variant_multi_turn_ = true;
        variant_adj_rate_ = false;
        break;
      case 'e':
        variant_14bit_ = false;
        variant_multi_turn_ = false;
        variant_adj_rate_ = true;
        break;
      case 'f':
        variant_14bit_ = true;
        variant_multi_turn_ = false;
        variant_adj_rate_ = true;
        break;
      case 'g':
        variant_14bit_ = false;
        variant_multi_turn_ = true;
        variant_adj_rate_ = true;
        break;
      case 'h':
        variant_14bit_ = true;
        variant_multi_turn_ = true;
        variant_adj_rate_ = true;
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(),
                     "Unrecognized AMT21[23] encoder model!");
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unrecognized AMT21 encoder model!");
  }

  node_->set_parameter(
      rclcpp::Parameter("encoder_overflow", !variant_multi_turn_));

  position_get_real_();
  velocity_last_position_ = position_last_;

  init_encoder_();
  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

int Interface::read_2_bytes(uint8_t addr, uint16_t &result) {
  std::string request;

  request.append((char *)&addr, 1);
  auto resp = prov_->query(2, request);
  RCLCPP_DEBUG(node_->get_logger(),
               "read_2_bytes(): serial_bus query returned %d bytes",
               (int)resp.length());

  if (resp.length() < 2) {
    return -1;
  }

  uint8_t low = resp[0];
  uint8_t high = resp[1] & 0x3F;
  uint8_t checksum = resp[1] >> 6;
  RCLCPP_DEBUG(node_->get_logger(),
               "read_2_bytes(): high %02x, low %02X, crc %02X", high, low,
               checksum);

  uint8_t odd = 1 ^ (1 & ((high >> 1) ^ (high >> 3) ^ (high >> 5) ^ (low >> 1) ^
                          (low >> 3) ^ (low >> 5) ^ (low >> 7)));
  uint8_t even = 1 ^ (1 & (high ^ (high >> 2) ^ (high >> 4) ^ low ^ (low >> 2) ^
                           (low >> 4) ^ (low >> 6)));
  if (checksum != ((odd << 1) | even)) {
    RCLCPP_INFO(node_->get_logger(), "read_2_bytes(): checksum error");
    return -1;
  }

  result = (high << 8) + low;
  if (!variant_14bit_) {
    result &= ~0x0003;
  }
  RCLCPP_DEBUG(node_->get_logger(),
               "read_2_bytes(): serial_bus query returned %04x", result);

  return 0;
}

void Interface::position_get_real_() {
  uint16_t position;
  RCLCPP_DEBUG(node_->get_logger(), "position_get_real_(): get position");
  if (read_2_bytes(addr_, position)) {
    // Return the last known good value if there is no valid response.
    return;
  }

  uint16_t turns = 0;
  if (variant_multi_turn_) {
    RCLCPP_DEBUG(node_->get_logger(), "position_get_real_(): get turns");
    if (read_2_bytes(addr_ + 1, turns)) {
      // Return the last known good value if there is no valid response.
      return;
    }
  }

  int32_t cumulative = (int32_t)(turns << 16) + (int32_t)position;
  position_last_ = ((double)cumulative) * 2.0 * M_PI / 65536.;
}

}  // namespace ros2_amt21
