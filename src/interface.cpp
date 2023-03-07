/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-04
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "encoder_amt21/interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>

#include "serial_bus/factory.hpp"

namespace encoder_amt21 {

Interface::Interface(rclcpp::Node *node)
    : remote_encoder::Implementation(node) {
  auto prefix = get_prefix_();

  prov_ = serial_bus::Factory::New(node);

  RCLCPP_DEBUG(node_->get_logger(),
               "Interface::Interface():"
               " started");

  node->declare_parameter("encoder_model", "amt212a");
  node->get_parameter("encoder_model", param_model);
  node->declare_parameter("encoder_amt21_addr", 84);
  node->get_parameter("encoder_amt21_addr", param_addr);

  std::string model = param_model.as_string();
  std::string model_normalized;
  for (auto &&ch : model) {
    if (ch != '-' && ch != '_' && ch != ' ') {
      model_normalized.push_back(tolower(ch));
    }
  }
  // There is no logic differences between amt212 and amt213.
  // They only offer by the physical form factor.
  if (model_normalized.rfind("amt212") == 0 ||
      model_normalized.rfind("amt213") == 0) {
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

  position_get_real_();
  velocity_last_position_ = position_last_;

  RCLCPP_DEBUG(node_->get_logger(), "Interface::Interface(): ended");
}

int Interface::read_2_bytes(uint8_t addr, uint16_t &result) {
  std::string request;

  request.append((char *)&addr, 1);
  auto resp = prov_->query(2, request);

  if (resp.length() < 2) {
  }

  uint8_t low = resp[0];
  uint8_t high = resp[1];

  uint8_t checksum = high >> 6;
  uint8_t odd = 1 ^ (1 & ((high >> 1) ^ (high >> 3) ^ (high >> 5) ^ (low >> 1) ^
                          (low >> 3) ^ (low >> 5) ^ (low >> 7)));
  uint8_t even = 1 ^ (1 & (high ^ (high >> 2) ^ (high >> 4) ^ low ^ (low >> 2) ^
                           (low >> 4) ^ (low >> 6)));
  if (checksum != ((odd << 1) | even)) {
    return -1;
  }

  result = ((high & 0x3F) << 8) + low;
  if (!variant_14bit_) {
    result &= ~0x0003;
  }

  return 0;
}

void Interface::position_get_real_() {
  uint16_t position;
  if (read_2_bytes(param_addr.as_int(), position)) {
    // Return the last known good value if there is no valid response.
    return;
  }
  uint16_t turns = 0;
  if (variant_multi_turn_) {
    if (read_2_bytes(param_addr.as_int() + 1, turns)) {
      // Return the last known good value if there is no valid response.
      return;
    }
  }

  int32_t cumulative = (int32_t)(turns << 16) + (int32_t)position;
  position_last_ = (double)cumulative / 65536.;
}

}  // namespace encoder_amt21
