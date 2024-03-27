/**
 * @file analytic_motion_hybrid.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "state_space/state_connector_hybrid/analytic_motion_hybrid.h"

#include <nav_utils/geometry_utils.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>

namespace state_space::state_connector_hybrid {

AnalyticMotionHybrid::AnalyticMotionHybrid(
    HybridModel&& hybrid_model,
    state_space::state_connector::StateConnectorParams&& connector_params,
    const std::shared_ptr<state_space::state_space_hybrid::StateSpaceHybrid>&
        state_space,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker)
    : hybrid_model_(std::move(hybrid_model)),
      connector_params_(std::move(connector_params)),
      state_space_(state_space),
      collision_checker_(collision_checker) {
  switch (hybrid_model_.hybrid_motion_model) {
    case HybridMotionModel::DUBINS:
      ompl_state_space_ = std::make_unique<ompl::base::DubinsStateSpace>(
          hybrid_model_.min_turning_radius);
      break;
    case HybridMotionModel::REEDS_SHEPP:
      ompl_state_space_ = std::make_unique<ompl::base::ReedsSheppStateSpace>(
          hybrid_model_.min_turning_radius);
      break;
    default:
      break;
  }
}

std::optional<AnalyticMotionHybrid::StateHybrid>
AnalyticMotionHybrid::tryAnalyticExpand(
    const StateHybrid& start, const StateHybrid& target) const {
  if (!isMotionModelValid()) {
    return std::nullopt;
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  StateHybridToOMPLState(from, start);
  StateHybridToOMPLState(to, target);

  int intervals = computeConnectionStatesNum(from, to);
  int iterations = std::min(connector_params_.max_extension_states, intervals);
  std::vector<double> reals;

  for (double i = 1.0; i <= iterations; i++) {
    ompl_state_space_->interpolate(from(), to(), i / intervals, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);

    if (poseInCollision(reals[0], reals[1], reals[2])) {
      return std::nullopt;
    }
  }

  return std::make_optional<StateHybrid>(
      reals[0], reals[1], state_space_->getClosestAngularBin(reals[2]));
}

bool AnalyticMotionHybrid::tryAnalyticConnect(
    const StateHybrid& start, const StateHybrid& goal) const {
  if (!isMotionModelValid()) {
    return false;
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  StateHybridToOMPLState(from, start);
  StateHybridToOMPLState(to, goal);

  int iterations = computeConnectionStatesNum(from, to);
  std::vector<double> reals;

  for (double i = 1.0; i <= iterations; i++) {
    ompl_state_space_->interpolate(from(), to(), i / iterations, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);

    if (poseInCollision(reals[0], reals[1], reals[2])) {
      return false;
    }
  }

  return true;
}

std::vector<AnalyticMotionHybrid::StateHybrid>
AnalyticMotionHybrid::getAnalyticPath(const StateHybrid& start,
                                      const StateHybrid& goal) const {
  if (!isMotionModelValid()) {
    return {};
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  StateHybridToOMPLState(from, start);
  StateHybridToOMPLState(to, goal);

  int intervals = computeConnectionStatesNum(from, to);
  std::vector<StateHybrid> path;
  path.reserve(intervals + 1);
  std::vector<double> reals;

  for (double i = 1.0; i <= intervals; i++) {
    ompl_state_space_->interpolate(from(), to(), i / intervals, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);
    path.emplace_back(reals[0], reals[1],
                      state_space_->getClosestAngularBin(reals[2]));
  }

  return path;
}

double AnalyticMotionHybrid::getAnalyticPathLength(
    const StateHybrid& start, const StateHybrid& goal) const {
  if (!isMotionModelValid()) {
    std::numeric_limits<double>::max();
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_);

  StateHybridToOMPLState(from, start);
  StateHybridToOMPLState(to, goal);

  return getAnalyticStateDistance(from, to);
}

int AnalyticMotionHybrid::computeConnectionStatesNum(
    ompl::base::ScopedState<>& start, ompl::base::ScopedState<>& goal) const {
  double d = getAnalyticStateDistance(start, goal);
  static const double sqrt_2 = std::sqrt(2.0);
  return std::floor(d / sqrt_2);
}

void AnalyticMotionHybrid::StateHybridToOMPLState(
    ompl::base::ScopedState<>& ompl_state, const StateHybrid& state) const {
  ompl_state[0] = state.x;
  ompl_state[1] = state.y;
  ompl_state[2] = state_space_->getAngleFromBin(state.theta);
}

bool AnalyticMotionHybrid::poseInCollision(double x, double y,
                                           double yaw) const {
  return collision_checker_->poseInCollision(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y), yaw,
      connector_params_.lethal_cost, connector_params_.allow_unknown);
}
}  // namespace state_space::state_connector_hybrid
