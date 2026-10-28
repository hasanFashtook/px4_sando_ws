// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_sando_bridge/px4_sando_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_sando_bridge::Px4SandoBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
