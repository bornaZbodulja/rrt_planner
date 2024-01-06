/**
 * @file state_connector_2d.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "state_space/state_connector_2d/state_connector_2d.h"

namespace state_space::state_connector_2d {

StateConnector2D::StateConnector2D(const ConnectorParamsT& params,
                                   const CollisionCheckerPtr& collision_checker)
    : state_space::state_connector::StateConnector<
          state_space::state_space_2d::State2D>(params, collision_checker),
      line_expander_(std::make_unique<LineConnector>()) {}

StateConnector2D::ExpansionResultT StateConnector2D::expandState(
    const StateT& current_state, const StateT& target_state) const {
  return line_expander_->tryLineExpand(current_state, target_state, params_,
                                       collision_checker_);
}

bool StateConnector2D::tryConnectStates(const StateT& start_state,
                                        const StateT& goal_state) const {
  return line_expander_->tryConnectStates(start_state, goal_state, params_,
                                          collision_checker_);
}

StateConnector2D::StateVector StateConnector2D::connectStates(
    const StateT& start_state, const StateT& goal_state) const {
  return line_expander_->getLinePath(start_state, goal_state);
}

double StateConnector2D::getStatesDistance(const StateT& start_state,
                                           const StateT& goal_state) const {
  return line_expander_->getLinePathLength(start_state, goal_state);
}
}  // namespace state_space::state_connector_2d
