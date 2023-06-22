/**
 * @file hybrid_motion_iterator.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Hybrid motion iterator implementation
 * @version 0.1
 * @date 2023-06-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__HYBRID_MOTION_ITERATOR_H_
#define RRT_PLANNER__HYBRID_MOTION_ITERATOR_H_

#include <cmath>

#include "nav_utils/geometry_utils.h"
#include "rrt_planner/hybrid_motion_constants.h"

namespace rrt_planner {

// TODO: Add support for Dubins (no reversing)
/**
 * @brief Iterator for motion between to hybrid poses
 */
class HybridMotionIterator {
 public:
  /**
   * @brief Constructor for motion iterator
   * @param x_s X position of start
   * @param y_s Y position of start
   * @param x_g X position of goal
   * @param y_g Y position of goal
   * @param yaw Initial heading of the vehicle
   * @param min_turning_radius Min turning radius of the vehicle
   * @param max_iterating_length
   */
  HybridMotionIterator(
      const double& x_s, const double& y_s, const double& x_g,
      const double& y_g, const double& yaw, const double& min_turning_radius,
      const int& max_iterating_length = std::numeric_limits<int>::max());

  /**
   * @brief
   * @return true
   * @return false
   */
  bool IsValid() const;

  /**
   * @brief Iterates to next pose
   */
  void Advance();

  /**
   * @brief Get X position of current iterating pose
   * @return double
   */
  inline double GetCurrentX() const { return x_; }

  /**
   * @brief Gets Y position of current iterating pose
   * @return double
   */
  inline double GetCurrentY() const { return y_; }

  /**
   * @brief Gets heading of current iterating pose
   * @return double
   */
  inline double GetCurrentYaw() const { return yaw_; }

  /**
   * @brief Gets X position of previous iterating pose
   * @return double
   */
  inline double GetPreviousX() const { return prev_x_; }

  /**
   * @brief Gets Y position of previous iterating pose
   * @return double
   */
  inline double GetPreviousY() const { return prev_y_; }

  /**
   * @brief Gets heading of previous iterating pose
   * @return double
   */
  inline double GetPreviousYaw() const { return prev_yaw_; }

 private:
  /**
   * @brief Initializes turning radius and motion type for iterator
   * @param euclidean_distance Euclidean distance between start and goal
   * @param heading_diff Heading distance between start and goal
   * @param min_turning_radius Min turning radius of the vehicle
   */
  void InitializeMotion(const double& euclidean_distance,
                        const double& heading_diff,
                        const double& min_turning_radius);

  /**
   * @brief Saves previous iterating pose
   */
  inline void SavePreviousState() {
    prev_x_ = x_;
    prev_y_ = y_;
    prev_yaw_ = yaw_;
  }

  // Current x, y, yaw
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  // Previous x, y, yaw
  double prev_x_{0.0}, prev_y_{0.0}, prev_yaw_{0.0};
  // Turning radius
  double r_{0.0};
  // Angle increment
  double angle_increment_{0.0};
  // Hybrid motion type
  HybridMotionType motion_type_{HybridMotionType::UNDEFINED};
  // Max number of iterations
  int max_iterations_{0};
  // Current number of iterations
  int iterations_{0};
};

}  // namespace rrt_planner

#endif
