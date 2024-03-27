/**
 * @file state_connector_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State connector hybrid implementation
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_HYBRID__STATE_CONNECTOR_HYBRID_H_
#define STATE_SPACE__STATE_CONNECTOR_HYBRID__STATE_CONNECTOR_HYBRID_H_

#include <nav_utils/collision_checker.h>

#include <memory>
#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector.h"
#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_connector_hybrid/analytic_motion_hybrid.h"
#include "state_space/state_connector_hybrid/model_hybrid.h"
#include "state_space/state_space_hybrid/state_hybrid.h"
#include "state_space/state_space_hybrid/state_space_hybrid.h"

namespace state_space::state_connector_hybrid {
class StateConnectorHybrid
    : public state_space::state_connector::StateConnector<
          state_space::state_space_hybrid::StateHybrid> {
 public:
  using StateHybrid = state_space::state_space_hybrid::StateHybrid;

  StateConnectorHybrid(
      HybridModel&& hybrid_model,
      state_space::state_connector::StateConnectorParams&& params,
      const std::shared_ptr<state_space::state_space_hybrid::StateSpaceHybrid>&
          state_space,
      const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker)
      : analytic_connector_(std::make_unique<AnalyticMotionHybrid>(
            std::move(hybrid_model), std::move(params), state_space,
            collision_checker)) {}

  ~StateConnectorHybrid() override = default;

  std::optional<StateHybrid> expandState(
      const StateHybrid& current_state,
      const StateHybrid& target_state) const override {
    return analytic_connector_->tryAnalyticExpand(current_state, target_state);
  }

  bool tryConnectStates(const StateHybrid& start_state,
                        const StateHybrid& goal_state) const override {
    return analytic_connector_->tryAnalyticConnect(start_state, goal_state);
  }

  std::vector<StateHybrid> connectStates(
      const StateHybrid& start_state,
      const StateHybrid& goal_state) const override {
    return analytic_connector_->getAnalyticPath(start_state, goal_state);
  }

  double getStatesDistance(const StateHybrid& start_state,
                           const StateHybrid& goal_state) const override {
    return analytic_connector_->getAnalyticPathLength(start_state, goal_state);
  }

 private:
  // Analytic connector pointer
  std::unique_ptr<AnalyticMotionHybrid> analytic_connector_;
};

}  // namespace state_space::state_connector_hybrid

#endif
