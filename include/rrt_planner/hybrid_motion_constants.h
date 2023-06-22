/**
 * @file hybrid_motion.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Hybrid motion type implementation
 * @version 0.1
 * @date 2023-06-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__HYBRID_MOTION_CONSTANTS_H_
#define RRT_PLANNER__HYBRID_MOTION_CONSTANTS_H_

#include <cmath>
#include <limits>

namespace rrt_planner {

enum class HybridMotionType {
  FORWARD = 0,
  FORWARD_LEFT = 1,
  FORWARD_RIGHT = 2,
  BACKWARD = 3,
  BACKWARD_LEFT = 4,
  BACKWARD_RIGHT = 5,
  UNDEFINED = 6
};

/**
 * @brief Gets type of motion between poses
 * @param heading_diff Heading difference between poses
 * @return HybridMotionType
 */
inline HybridMotionType GetHybridMotionType(const double& heading_diff) {
  if (heading_diff == 0.0) {
    return HybridMotionType::FORWARD;
  } else if (heading_diff > 0.0 && heading_diff <= M_PI_2) {
    return HybridMotionType::FORWARD_LEFT;
  } else if (heading_diff > M_PI_2 && heading_diff < M_PI) {
    return HybridMotionType::BACKWARD_LEFT;
  } else if (heading_diff == M_PI) {
    return HybridMotionType::BACKWARD;
  } else if (heading_diff > M_PI && heading_diff <= 3 * M_PI_2) {
    return HybridMotionType::BACKWARD_RIGHT;
  } else if (heading_diff > 3 * M_PI_2 && heading_diff < 2 * M_PI) {
    return HybridMotionType::FORWARD_RIGHT;
  } else {
    return HybridMotionType::UNDEFINED;
  }
}

/**
 * @brief Whether motion type is straight forward or backward
 * @param motion_type
 * @return True if motion is straight, false otherwise
 */
inline bool IsMotionStraight(const HybridMotionType& motion_type) {
  return motion_type == HybridMotionType::FORWARD ||
         motion_type == HybridMotionType::BACKWARD;
}

/**
 * @brief Computes turning radius for the motion between two poses
 * @param motion_type Hybrid motion type
 * @param euclidean_distance Euclidean distance between poses
 * @param heading_diff Heading difference between poses
 * @return double
 */
inline double ComputeTurningRadius(const HybridMotionType& motion_type,
                                   const double& euclidean_distance,
                                   const double& heading_diff) {
  if (IsMotionStraight(motion_type)) {
    return std::numeric_limits<double>::infinity();
  } else {
    return euclidean_distance / (2.0 * std::abs(std::sin(heading_diff)));
  }
}

/**
 * @brief Computes arc curve length for the motion between poses
 * @param motion_type Hybrid motion type
 * @param euclidean_distance Euclidean distance between poses
 * @param heading_diff Heading difference between poses
 * @param turning_radius Turning radius for the motion
 * @return double
 */
inline double ComputeArcLength(const HybridMotionType& motion_type,
                               const double& euclidean_distance,
                               const double& heading_diff,
                               const double& turning_radius) {
  if (IsMotionStraight(motion_type)) {
    return euclidean_distance;
  } else {
    return turning_radius * heading_diff;
  }
}

}  // namespace rrt_planner

#endif
