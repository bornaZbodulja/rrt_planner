/**
 * @file state_connector_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State connector 2D implementation
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_2D__STATE_CONNECTOR_2D_H_
#define STATE_SPACE__STATE_CONNECTOR_2D__STATE_CONNECTOR_2D_H_

#include <nav_utils/collision_checker.h>

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
  using State2D = state_space::state_space_2d::State2D;

  StateConnector2D(
      state_space::state_connector::StateConnectorParams&& params,
      const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker)
      : state_space::state_connector::StateConnector<
            state_space::state_space_2d::State2D>(),
        line_connector_(std::make_unique<LineConnector>(std::move(params),
                                                        collision_checker)) {}

  ~StateConnector2D() override = default;

  std::optional<State2D> expandState(
      const State2D& current_state,
      const State2D& target_state) const override {
    return line_connector_->tryLineExpand(current_state, target_state);
  }

  bool tryConnectStates(const State2D& start_state,
                        const State2D& goal_state) const override {
    return line_connector_->tryConnectStates(start_state, goal_state);
  }

  std::vector<State2D> connectStates(const State2D& start_state,
                                     const State2D& goal_state) const override {
    return line_connector_->getLinePath(start_state, goal_state);
  }

  double getStatesDistance(const State2D& start_state,
                           const State2D& goal_state) const override {
    return line_connector_->getLinePathLength(start_state, goal_state);
  }

 private:
  // Line connector pointer
  std::unique_ptr<LineConnector> line_connector_;
};
}  // namespace state_space::state_connector_2d

#endif  // STATE_SPACE__STATE_CONNECTOR_2D__STATE_CONNECTOR_2D_H_
