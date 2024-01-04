/**
 * @file state_connector.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State connector interface
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR__STATE_CONNECTOR_H_
#define STATE_SPACE__STATE_CONNECTOR__STATE_CONNECTOR_H_

#include <nav_utils/nav_utils.h>

#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector_params.h"

namespace state_space::state_connector {
/**
 * @brief State connector interface
 * @tparam StateT State template
 */
template <typename StateT>
class StateConnector {
 public:
  using StateVector = std::vector<StateT>;
  using ExpansionResultT = std::optional<StateT>;

  StateConnector(const StateConnectorParams& params,
                 const CollisionCheckerPtr& collision_checker)
      : params_(params), collision_checker_(collision_checker) {}
  virtual ~StateConnector() = default;

  void updateCollisionChecker(const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

  /**
   * @brief Expands current state towards target state for max extension states
   * @param current_state
   * @param target_state
   * @return ExpansionResultT
   */
  virtual ExpansionResultT expandState(const StateT& current_state,
                                       const StateT& target_state) const = 0;

  /**
   * @brief Tries connecting start and goal state
   * @param start_state
   * @param goal_state
   * @return True if connection is valid, false otherwise
   */
  virtual bool tryConnectStates(const StateT& start_state,
                                const StateT& goal_state) const = 0;

  /**
   * @brief Returns path connecting start and goal state
   * @param start_state
   * @param goal_state
   * @return StateVector
   */
  virtual StateVector connectStates(const StateT& start_state,
                                    const StateT& goal_state) const = 0;

  /**
   * @brief Returns length of path connecting two states
   * @param start_state
   * @param goal_state
   * @return double
   */
  virtual double getStatesDistance(const StateT& start_state,
                                   const StateT& goal_state) const = 0;

 protected:
  // State connector parameters
  StateConnectorParams params_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
};

}  // namespace state_space::state_connector

#endif
