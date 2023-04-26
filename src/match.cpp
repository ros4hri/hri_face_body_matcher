// Copyright 2023 PAL Robotics S.L.
// All Rights Reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "face_body_matcher.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_face_body_matcher");
  FaceBodyMatcher node{};
  ros::spin();
  return 0;
}
