// Copyright 2023 PAL Robotics S.L.
// All Rights Reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "face_body_matcher.hpp"

#include <dlib/matrix.h>
#include <dlib/optimization/max_cost_assignment.h>
#include <hri/body.h>
#include <hri/face.h>
#include <hri/hri.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/Skeleton2D.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

FaceBodyMatcher::FaceBodyMatcher()
{
  ros::param::param<double>("~confidence_threshold", confidence_threshold_, 0.5);
  ros::param::param<double>("~confidence_scaling_factor", confidence_scaling_factor_, 2.);
  ROS_INFO("Parameter 'confidence_threshold': %f", confidence_threshold_);
  ROS_INFO("Parameter 'confidence_scaling_factor': %f", confidence_scaling_factor_);

  match_timer_ = node_handle_.createTimer(ros::Duration(0.1), std::bind(&FaceBodyMatcher::match, this));
  match_pub_ = node_handle_.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 10, false);
}

void FaceBodyMatcher::match()
{
  // Using the Hungarian algorithm to find the maximum cost assignment between bodies and faces.
  // The cost is represented by the likelihood of the assignment in a scale from 0 to 100.
  // Since the algorithm requires a balanced assignment (number of bodies equal to the number of faces) which is not
  // guaranteed, dummy elements are added to the smaller set with zero likelihood associated.
  // See http://dlib.net/max_cost_assignment_ex.cpp.html.

  update_hri_data();

  auto body_and_face_exist = [&](int row, int col) -> bool {
    return (row < static_cast<int>(body_ids_.size())) && (col < static_cast<int>(face_ids_.size()));
  };

  int dim{ static_cast<int>(std::max(bodies_.size(), faces_.size())) };
  dlib::matrix<int> cost(dim, dim);
  for (int row = 0; row < dim; ++row)
  {
    for (int col = 0; col < dim; ++col)
    {
      if (body_and_face_exist(row, col))
      {
        cost(row, col) = matching_confidence(bodies_[body_ids_[row]], faces_[face_ids_[col]]);
      }
      else
      {
        cost(row, col) = 0;
      }
    }
  }

  auto assignment{ dlib::max_cost_assignment(cost) };

  for (int row = 0; row < dim; ++row)
  {
    auto matched_col{ assignment[row] };
    if (body_and_face_exist(row, matched_col))
    {
      publish_match(body_ids_[row], face_ids_[matched_col], cost(row, matched_col) / 100.);
    }
  }
}

int FaceBodyMatcher::matching_confidence(hri::BodyWeakConstPtr body, hri::FaceWeakConstPtr face)
{
  // It returns the confidence of the face to body matching in a scale from 0 to 100, based on face center to body nose
  // distance. It is 100 if the points coincide and decreases linearly to 50 when the points distance is equal to the
  // the face RoI diagonal length scaled by the "confidence scaling factor".

  int confidence{};
  auto body_skeleton{ body.lock()->skeleton() };
  auto face_roi{ face.lock()->roi() };
  if (!body_skeleton.empty() && (face_roi != NormROI()))
  {
    auto body_nose{ body_skeleton[hri_msgs::Skeleton2D::NOSE] };
    cv::Point2f body_nose_point{ body_nose.x, body_nose.y };
    cv::Point2f face_top_left_point{ face_roi.xmin, face_roi.ymin };
    cv::Point2f face_bot_right_point{ face_roi.xmax, face_roi.ymax };
    cv::Point2f face_center_point{ (face_top_left_point + face_bot_right_point) / 2. };

    auto distance{ cv::norm(body_nose_point - face_center_point) };
    auto half_confidence_distance{ cv::norm(face_bot_right_point - face_top_left_point) / confidence_scaling_factor_ };

    confidence = static_cast<int>(std::max(0., 100. - (50. * distance / half_confidence_distance)));
  }
  return confidence;
}

void FaceBodyMatcher::publish_match(hri::ID body_id, hri::ID face_id, double confidence)
{
  if (confidence > confidence_threshold_)
  {
    hri_msgs::IdsMatch match_msg{};
    match_msg.id1 = body_id;
    match_msg.id1_type = match_msg.BODY;
    match_msg.id2 = face_id;
    match_msg.id2_type = match_msg.FACE;
    match_msg.confidence = confidence;

    match_pub_.publish(match_msg);
  }
}

void FaceBodyMatcher::update_hri_data()
{
  bodies_ = hri_listener_.getBodies();
  faces_ = hri_listener_.getFaces();

  body_ids_.clear();
  body_ids_.reserve(bodies_.size());
  for (const auto& key_value : bodies_)
  {
    body_ids_.push_back(key_value.first);
  }

  face_ids_.clear();
  face_ids_.reserve(faces_.size());
  for (const auto& key_value : faces_)
  {
    face_ids_.push_back(key_value.first);
  }
}