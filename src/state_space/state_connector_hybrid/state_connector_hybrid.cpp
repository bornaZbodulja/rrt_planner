/**
 * @file state_connector_hybrid.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "state_space/state_connector_hybrid/state_connector_hybrid.h"

using namespace state_space::state_connector_hybrid;

StateConnectorHybrid::StateConnectorHybrid(
    const StateSpacePtr& state_space, const HybridModel& hybrid_model,
    const ConnectorParamsT& params,
    const CollisionCheckerPtr& collision_checker)
    : state_space::state_connector::StateConnector<
          state_space::state_space_hybrid::StateHybrid>(params,
                                                        collision_checker),
      state_space_(state_space),
      analytic_expander_(std::make_unique<AnalyticMotionHybrid>(hybrid_model)) {
}

StateConnectorHybrid::ExpansionResultT StateConnectorHybrid::expandState(
    const StateT& current_state, const StateT& target_state) const {
  return analytic_expander_->tryAnalyticExpand(current_state, target_state,
                                               state_space_.get(), params_,
                                               collision_checker_);
}

bool StateConnectorHybrid::tryConnectStates(const StateT& start_state,
                                            const StateT& goal_state) const {
  return analytic_expander_->tryAnalyticConnect(
      start_state, goal_state, state_space_.get(), params_, collision_checker_);
}

StateConnectorHybrid::StateVector StateConnectorHybrid::connectStates(
    const StateT& start_state, const StateT& goal_state) const {
  return analytic_expander_->getAnalyticPath(start_state, goal_state,
                                             state_space_.get());
}

double StateConnectorHybrid::getStatesDistance(const StateT& start_state,
                                               const StateT& goal_state) const {
  return analytic_expander_->getAnalyticPathLength(start_state, goal_state,
                                                   state_space_.get());
}
