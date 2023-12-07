/**
 * @file state_connector_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_HYBRID__STATE_CONNECTOR_HYBRID_H_
#define STATE_SPACE__STATE_CONNECTOR_HYBRID__STATE_CONNECTOR_HYBRID_H_

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
  using StateT = state_space::state_space_hybrid::StateHybrid;
  using StateSpaceT = state_space::state_space_hybrid::StateSpaceHybrid;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;
  using StateVector = std::vector<StateT>;
  using AnalyticExpanderPtr = std::unique_ptr<AnalyticMotionHybrid>;
  using ExpansionResultT = std::optional<StateT>;
  using ConnectorParamsT = state_space::state_connector::StateConnectorParams;
  using state_space::state_connector::StateConnector<
      StateT>::collision_checker_;
  using state_space::state_connector::StateConnector<StateT>::params_;

  StateConnectorHybrid(const StateSpacePtr& state_space,
                       const HybridModel& hybrid_model,
                       const ConnectorParamsT& params,
                       const CollisionCheckerPtr& collision_checker);

  ExpansionResultT expandState(const StateT& current_state,
                               const StateT& target_state) const override;

  bool tryConnectStates(const StateT& start_state,
                        const StateT& goal_state) const override;

  StateVector connectStates(const StateT& start_state,
                            const StateT& goal_state) const override;

  double getStatesDistance(const StateT& start_state,
                           const StateT& goal_state) const override;

 protected:
  // State space pointer
  StateSpacePtr state_space_;
  // Analytic expander pointer
  AnalyticExpanderPtr analytic_expander_;
};

}  // namespace state_space::state_connector_hybrid

#endif
