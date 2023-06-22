/**
 * @file hybrid_motion_iterator.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-06-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/hybrid_motion_iterator.h"

using namespace rrt_planner;

HybridMotionIterator::HybridMotionIterator(const double& x_s, const double& y_s,
                                           const double& x_g, const double& y_g,
                                           const double& yaw,
                                           const double& min_turning_radius,
                                           const int& max_iterating_length)
    : x_(x_s),
      y_(y_s),
      prev_x_(x_s),
      prev_y_(y_s),
      max_iterations_(max_iterating_length) {
  auto start_heading = yaw;
  auto goal_heading = std::atan2(y_g - y_s, x_g - x_s);
  // Euclidean distance between start and goal
  const auto b = nav_utils::GetEuclideanDistance(x_g, y_g, x_s, y_s);
  // Heading difference
  const auto heading_diff =
      nav_utils::GetHeadingDifference(goal_heading, start_heading);
  InitializeMotion(b, heading_diff, min_turning_radius);
}

bool HybridMotionIterator::IsValid() const {
  return iterations_ < max_iterations_ &&
         motion_type_ != HybridMotionType::UNDEFINED;
}

void HybridMotionIterator::Advance() {
  iterations_++;
  SavePreviousState();

  switch (motion_type_) {
    case HybridMotionType::FORWARD:
      x_ += std::cos(yaw_);
      y_ += std::sin(yaw_);
      break;
    case HybridMotionType::FORWARD_LEFT:
      x_ += r_ * (std::sin(yaw_ + angle_increment_) - std::sin(yaw_));
      y_ += r_ * (std::cos(yaw_) - std::cos(yaw_ + angle_increment_));
      yaw_ += angle_increment_;
      nav_utils::NormalizeAngle(yaw_);
      break;
    case HybridMotionType::FORWARD_RIGHT:
      x_ += r_ * (std::sin(yaw_) - std::sin(yaw_ - angle_increment_));
      y_ += r_ * (std::cos(yaw_ - angle_increment_) - std::cos(yaw_));
      yaw_ -= angle_increment_;
      nav_utils::NormalizeAngle(yaw_);
      break;
    case HybridMotionType::BACKWARD:
      x_ -= std::cos(yaw_);
      y_ -= std::sin(yaw_);
      break;
    case HybridMotionType::BACKWARD_LEFT:
      x_ += r_ * (std::sin(yaw_ - angle_increment_) - std::sin(yaw_));
      y_ += r_ * (std::cos(yaw_) - std::cos(yaw_ - angle_increment_));
      yaw_ -= angle_increment_;
      nav_utils::NormalizeAngle(yaw_);
      break;
    case HybridMotionType::BACKWARD_RIGHT:
      x_ += r_ * (std::sin(yaw_) - std::sin(yaw_ + angle_increment_));
      y_ += r_ * (std::cos(yaw_ + angle_increment_) - std::cos(yaw_));
      yaw_ += angle_increment_;
      nav_utils::NormalizeAngle(yaw_);
      break;
    default:
      break;
  }
}

void HybridMotionIterator::InitializeMotion(const double& euclidean_distance,
                                            const double& heading_diff,
                                            const double& min_turning_radius) {
  motion_type_ = GetHybridMotionType(heading_diff);
  r_ = ComputeTurningRadius(motion_type_, euclidean_distance, heading_diff);
  if (r_ < min_turning_radius) {
    r_ = min_turning_radius;
  }

  if (IsMotionStraight(motion_type_)) {
    angle_increment_ = 0.0;
  } else {
    angle_increment_ = 1.0 / r_;
  }
}
