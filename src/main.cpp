// Copyright 2024 PAL Robotics S.L.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "rclcpp/rclcpp.hpp"

#include "hri_face_body_matcher/node_face_body_matcher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hri_face_body_matcher::NodeFaceBodyMatcher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
