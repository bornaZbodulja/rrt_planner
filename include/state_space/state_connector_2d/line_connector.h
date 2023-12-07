/**
 * @file line_connector.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Line connector for 2D expansion
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_2D__LINE_CONNECTOR_H_
#define STATE_SPACE__STATE_CONNECTOR_2D__LINE_CONNECTOR_H_

#include <nav_utils/nav_utils.h>

#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_space_2d/state_2d.h"

namespace state_space::state_connector_2d {

class LineConnector {
 public:
  using StateT = state_space::state_space_2d::State2D;
  using StateVector = std::vector<StateT>;
  using ExpansionResultT = std::optional<StateT>;
  using ConnectionParamsT = state_space::state_connector::StateConnectorParams;

  LineConnector() = default;
  ~LineConnector() = default;

  /**
   * @brief Tries straight line expanding state towards target state
   * @param start
   * @param target
   * @param connection_params
   * @param collision_checker
   * @return ExpansionResultT
   */
  ExpansionResultT tryLineExpand(
      const StateT& start, const StateT& target,
      const ConnectionParamsT& connection_params,
      const CollisionCheckerPtr& collision_checker) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param connection_params
   * @param collision_checker
   * @return true
   * @return false
   */
  bool tryConnectStates(const StateT& start, const StateT& goal,
                        const ConnectionParamsT& connection_params,
                        const CollisionCheckerPtr& collision_checker) const;

  StateVector getLinePath(const StateT& start, const StateT& goal) const;

  double getLinePathLength(const StateT& start, const StateT& goal) const;

 private:
  LineIteratorT createLineIterator(const StateT& start,
                                   const StateT& goal) const;
};

}  // namespace state_space::state_connector_2d

#endif
