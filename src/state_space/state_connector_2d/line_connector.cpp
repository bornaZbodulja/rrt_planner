/**
 * @file line_connector.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "state_space/state_connector_2d/line_connector.h"

namespace state_space::state_connector_2d {

LineConnector::ExpansionResultT LineConnector::tryLineExpand(
    const StateT& start, const StateT& target,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  LineIteratorT line_iterator = createLineIterator(start, target);
  // skip start position
  line_iterator.advance();

  // TODO: Simplify this

  int iteration{0};

  while (line_iterator.isValid() &&
         iteration < connection_params.max_extension_states) {
    if (collision_checker->pointInCollision(
            line_iterator.getCurrentX(), line_iterator.getCurrentY(),
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return std::nullopt;
    }
    iteration++;
    line_iterator.advance();
  }

  return std::make_optional<StateT>(
      static_cast<double>(line_iterator.getPreviousX()),
      static_cast<double>(line_iterator.getPreviousY()));
}

bool LineConnector::tryConnectStates(
    const StateT& start, const StateT& goal,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  LineIteratorT line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.advance();

  // TODO: Simplify this

  while (line_iterator.isValid()) {
    if (collision_checker->pointInCollision(
            line_iterator.getCurrentX(), line_iterator.getCurrentY(),
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return false;
    }
    line_iterator.advance();
  }

  return true;
}

LineConnector::StateVector LineConnector::getLinePath(
    const StateT& start, const StateT& goal) const {
  StateVector path;

  LineIteratorT line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.advance();

  while (line_iterator.isValid()) {
    path.emplace_back(static_cast<double>(line_iterator.getCurrentX()),
                      static_cast<double>(line_iterator.getCurrentY()));
    line_iterator.advance();
  }

  return path;
}

double LineConnector::getLinePathLength(const StateT& start,
                                        const StateT& goal) const {
  return std::hypot(goal.x - start.x, goal.y - start.y);
}

typename LineConnector::LineIteratorT LineConnector::createLineIterator(
    const StateT& start, const StateT& goal) const {
  return LineIteratorT{static_cast<int>(start.x), static_cast<int>(start.y),
                       static_cast<int>(goal.x), static_cast<int>(goal.y)};
}
}  // namespace state_space::state_connector_2d
