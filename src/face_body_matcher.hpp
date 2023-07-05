// Copyright 2023 PAL Robotics S.L.
// All Rights Reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#ifndef FACE_BODY_MATCHER_H
#define FACE_BODY_MATCHER_H

#include <hri/hri.h>
#include <ros/ros.h>

#include <map>
#include <vector>

class FaceBodyMatcher
{
public:
  FaceBodyMatcher();

private:
  void match();
  int matching_confidence(hri::BodyWeakConstPtr body, hri::FaceWeakConstPtr face);
  void publish_match(hri::ID body_id, hri::ID face_id, double confidence);
  void update_hri_data();

  ros::NodeHandle node_handle_{};
  hri::HRIListener hri_listener_{};
  ros::Timer match_timer_{};
  ros::Publisher match_pub_{};
  std::map<hri::ID, hri::BodyWeakConstPtr> bodies_{};
  std::map<hri::ID, hri::FaceWeakConstPtr> faces_{};
  std::vector<hri::ID> body_ids_{};
  std::vector<hri::ID> face_ids_{};
  double confidence_threshold_{};       // lower confidence matches are not published
  double confidence_scaling_factor_{};  // face to body face distance, relative to face diagonal,
                                        // correspondent to 0.5 confidence
};

#endif  // FACE_BODY_MATCHER_H
