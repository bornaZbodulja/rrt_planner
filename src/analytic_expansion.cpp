/**
 * @file analytic_expansion.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-06-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/analytic_expansion.h"

#include <ompl/base/ScopedState.h>

#include "nav_utils/geometry_utils.h"

using namespace rrt_planner;

template <typename NodeT>
typename AnalyticExpansion<NodeT>::ExpansionResult
AnalyticExpansion<NodeT>::TryAnalyticExpansion(const Coordinates& start,
                                               const Coordinates& goal,
                                               const NodePtr& node,
                                               const unsigned char& lethal_cost,
                                               const bool& allow_unknown,
                                               const int& max_length) {
  if (motion_model_ != MotionModel::DUBINS &&
      motion_model_ != MotionModel::REEDS_SHEPP) {
    return {};
  }

  static ompl::base::ScopedState<> from(node->motion_table.state_space),
      to(node->motion_table.state_space), s(node->motion_table.state_space);

  from[0] = start.x;
  from[1] = start.y;
  from[2] = node->motion_table.GetAngleFromBin(start.theta);
  to[0] = goal.x;
  to[1] = goal.y;
  to[2] = node->motion_table.GetAngleFromBin(goal.theta);

  const auto d = node->motion_table.state_space->distance(from(), to());
  static const double sqrt_2 = std::sqrt(2.0);
  int intervals = std::floor(d / sqrt_2);
  int iterations = (max_length > intervals) ? intervals : max_length;
  int angle{0};
  Coordinates coordinates{};
  bool pose_in_collision{false};
  std::vector<double> reals;

  for (double i = 0.0; i <= iterations; i++) {
    node->motion_table.state_space->interpolate(from(), to(), i / intervals,
                                                s());
    reals = s.reals();
    nav_utils::NormalizeAngle(reals[2]);
    angle = node->motion_table.GetClosestAngularBin(reals[2]);
    coordinates = {static_cast<int>(reals[0]), static_cast<int>(reals[1]),
                   angle};
    pose_in_collision = collision_checker_->PoseInCollision(
        static_cast<unsigned int>(coordinates.x),
        static_cast<unsigned int>(coordinates.y), reals[2], lethal_cost,
        allow_unknown);

    if (pose_in_collision) {
      return {};
    }
  }

  return std::make_optional(coordinates);
}

template <typename NodeT>
double AnalyticExpansion<NodeT>::GetAnalyticPathLength(
    const Coordinates& start, const Coordinates& goal,
    const NodePtr& node) const {
  if (motion_model_ != MotionModel::DUBINS &&
      motion_model_ != MotionModel::REEDS_SHEPP) {
    return std::numeric_limits<double>::max();
  }

  static ompl::base::ScopedState<> from(node->motion_table.state_space),
      to(node->motion_table.state_space);

  from[0] = start.x;
  from[1] = start.y;
  from[2] = node->motion_table.GetAngleFromBin(start.theta);
  to[0] = goal.x;
  to[1] = goal.y;
  to[2] = node->motion_table.GetAngleFromBin(goal.theta);

  return node->motion_table.state_space->distance(from(), to());
}

template <>
typename AnalyticExpansion<Node2D>::ExpansionResult
AnalyticExpansion<Node2D>::TryAnalyticExpansion(
    const Coordinates& start, const Coordinates& goal, const NodePtr& node,
    const unsigned char& lethal_cost, const bool& allow_unknown,
    const int& max_length) {
  return {};
}

template <>
double AnalyticExpansion<Node2D>::GetAnalyticPathLength(
    const Coordinates& start, const Coordinates& goal,
    const NodePtr& node) const {
  return std::numeric_limits<double>::max();
}

// Instantiate algorithm for the supported template types
template class AnalyticExpansion<Node2D>;
template class AnalyticExpansion<NodeHybrid>;
