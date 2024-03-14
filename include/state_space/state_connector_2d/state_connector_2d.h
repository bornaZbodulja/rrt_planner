/**
 * @file state_connector_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_2D__STATE_CONNECTOR_2D_H_
#define STATE_SPACE__STATE_CONNECTOR_2D__STATE_CONNECTOR_2D_H_

#include <memory>
#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector.h"
#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_connector_2d/line_connector.h"
#include "state_space/state_space_2d/state_2d.h"

namespace state_space::state_connector_2d {
class StateConnector2D : public state_space::state_connector::StateConnector<
                             state_space::state_space_2d::State2D> {
 public:
  using StateT = state_space::state_space_2d::State2D;
  using StateVector = std::vector<StateT>;
  using LineExpanderPtr = std::unique_ptr<LineConnector>;
  using ExpansionResultT = std::optional<StateT>;
  using ConnectorParamsT = state_space::state_connector::StateConnectorParams;
  using state_space::state_connector::StateConnector<
      StateT>::collision_checker_;
  using state_space::state_connector::StateConnector<StateT>::params_;

  StateConnector2D(ConnectorParamsT&& params,
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
  // Line expander pointer
  LineExpanderPtr line_expander_;
};

}  // namespace state_space::state_connector_2d

#endif
