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

#include <nav_utils/collision_checker.h>
#include <nav_utils/line_iterator.h>

#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_space_2d/state_2d.h"

namespace state_space::state_connector_2d {

class LineConnector {
 public:
  using State2D = state_space::state_space_2d::State2D;

  explicit LineConnector(
      state_space::state_connector::StateConnectorParams&& connection_params,
      std::shared_ptr<nav_utils::CollisionChecker> collision_checker)
      : connector_params_(std::move(connection_params)),
        collision_checker_(collision_checker) {}

  ~LineConnector() = default;

  /**
   * @brief Tries straight line expanding state towards target state
   * @param start
   * @param target
   * @param connection_params
   * @param collision_checker
   * @return ExpansionResultT
   */
  std::optional<State2D> tryLineExpand(const State2D& start,
                                       const State2D& target) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param connection_params
   * @param collision_checker
   * @return true
   * @return false
   */
  bool tryConnectStates(const State2D& start, const State2D& goal) const;

  std::vector<State2D> getLinePath(const State2D& start,
                                   const State2D& goal) const;

  double getLinePathLength(const State2D& start, const State2D& goal) const;

 private:
  nav_utils::LineIterator createLineIterator(const State2D& start,
                                             const State2D& goal) const;

  /**
   * @brief Check if point is in collision
   * @param x X coordinate
   * @param y Y coordinate
   * @return True if point is in collision, false otherwise
   */
  bool pointInCollision(unsigned int x, unsigned int y) const;

  // Connector parameters
  state_space::state_connector::StateConnectorParams connector_params_;
  // Collision checker pointer
  std::shared_ptr<nav_utils::CollisionChecker> collision_checker_;
};

}  // namespace state_space::state_connector_2d

#endif
