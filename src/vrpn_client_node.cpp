// Copyright 2015 Clearpath Robotics, Inc.
// Copyright 2026 TDC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the conditions of the BSD
// 3-Clause license are met.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vrpn_client_ros2/vrpn_client_ros2.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<vrpn_client_ros2::VrpnClientRos>();
  try {
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "%s", exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
