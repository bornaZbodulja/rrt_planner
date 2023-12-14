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

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "nav_utils/geometry_utils.h"

using namespace state_space::state_connector_hybrid;

AnalyticMotionHybrid::AnalyticMotionHybrid(const HybridModel& hybrid_model)
    : hybrid_model_(hybrid_model) {
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

AnalyticMotionHybrid::ExpansionResultT AnalyticMotionHybrid::tryAnalyticExpand(
    const StateT& start, const StateT& target, const StateSpacePtr& state_space,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  if (!isMotionModelValid()) {
    return {};
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  stateToOMPLState(from, start, state_space);
  stateToOMPLState(to, target, state_space);

  const auto intervals = computeConnectionStatesNum(from, to);
  const auto iterations = (connection_params.max_extension_states > intervals)
                              ? intervals
                              : connection_params.max_extension_states;
  std::vector<double> reals;

  for (double i = 1.0; i <= iterations; i++) {
    ompl_state_space_->interpolate(from(), to(), i / intervals, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);

    if (collision_checker->poseInCollision(
            static_cast<unsigned int>(reals[0]),
            static_cast<unsigned int>(reals[1]), reals[2],
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return {};
    }
  }

  return std::make_optional<StateT>(
      reals[0], reals[1], state_space->getClosestAngularBin(reals[2]));
}

bool AnalyticMotionHybrid::tryAnalyticConnect(
    const StateT& start, const StateT& goal, const StateSpacePtr& state_space,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  if (!isMotionModelValid()) {
    return {};
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  stateToOMPLState(from, start, state_space);
  stateToOMPLState(to, goal, state_space);

  const auto iterations = computeConnectionStatesNum(from, to);
  std::vector<double> reals;

  for (double i = 1.0; i <= iterations; i++) {
    ompl_state_space_->interpolate(from(), to(), i / iterations, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);

    if (collision_checker->poseInCollision(
            static_cast<unsigned int>(reals[0]),
            static_cast<unsigned int>(reals[1]), reals[2],
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return false;
    }
  }

  return true;
}

AnalyticMotionHybrid::StateVector AnalyticMotionHybrid::getAnalyticPath(
    const StateT& start, const StateT& goal,
    const StateSpacePtr& state_space) const {
  if (!isMotionModelValid()) {
    return {};
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_), s(ompl_state_space_);

  stateToOMPLState(from, start, state_space);
  stateToOMPLState(to, goal, state_space);

  const auto intervals = computeConnectionStatesNum(from, to);
  StateVector path;
  path.reserve(intervals + 1);
  std::vector<double> reals;

  for (double i = 1.0; i <= intervals; i++) {
    ompl_state_space_->interpolate(from(), to(), i / intervals, s());
    reals = s.reals();
    nav_utils::normalizeAngle(reals[2]);
    path.emplace_back(reals[0], reals[1],
                      state_space->getClosestAngularBin(reals[2]));
  }

  return path;
}

double AnalyticMotionHybrid::getAnalyticPathLength(
    const StateT& start, const StateT& goal,
    const StateSpacePtr& state_space) const {
  if (!isMotionModelValid()) {
    std::numeric_limits<double>::max();
  }

  static ompl::base::ScopedState<> from(ompl_state_space_),
      to(ompl_state_space_);

  stateToOMPLState(from, start, state_space);
  stateToOMPLState(to, goal, state_space);

  return getAnalyticStateDistance(from, to);
}

int AnalyticMotionHybrid::computeConnectionStatesNum(
    ompl::base::ScopedState<>& start, ompl::base::ScopedState<>& goal) const {
  const auto d = getAnalyticStateDistance(start, goal);
  static const double sqrt_2 = std::sqrt(2.0);
  return std::floor(d / sqrt_2);
}

void AnalyticMotionHybrid::stateToOMPLState(
    ompl::base::ScopedState<>& ompl_state, const StateT& state,
    const StateSpacePtr& state_space) const {
  ompl_state[0] = state.x;
  ompl_state[1] = state.y;
  ompl_state[2] = state_space->getAngleFromBin(state.theta);
}
