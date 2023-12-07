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

using namespace state_space::state_connector_2d;

LineConnector::ExpansionResultT LineConnector::tryLineExpand(
    const StateT& start, const StateT& target,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  auto&& line_iterator = createLineIterator(start, target);
  // skip start position
  line_iterator.Advance();

  // TODO: Simplify this

  int iteration{0};

  while (line_iterator.IsValid() &&
         iteration < connection_params.max_extension_states) {
    if (collision_checker->PointInCollision(
            line_iterator.GetCurrentX(), line_iterator.GetCurrentY(),
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return {};
    }
    iteration++;
    line_iterator.Advance();
  }

  return std::make_optional<StateT>(
      static_cast<double>(line_iterator.GetPreviousX()),
      static_cast<double>(line_iterator.GetPreviousY()));
}

bool LineConnector::tryConnectStates(
    const StateT& start, const StateT& goal,
    const ConnectionParamsT& connection_params,
    const CollisionCheckerPtr& collision_checker) const {
  auto&& line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.Advance();

  // TODO: Simplify this

  while (line_iterator.IsValid()) {
    if (collision_checker->PointInCollision(
            line_iterator.GetCurrentX(), line_iterator.GetCurrentY(),
            connection_params.lethal_cost, connection_params.allow_unknown)) {
      return false;
    }
    line_iterator.Advance();
  }

  return true;
}

LineConnector::StateVector LineConnector::getLinePath(
    const StateT& start, const StateT& goal) const {
  StateVector path;

  auto&& line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.Advance();

  while (line_iterator.IsValid()) {
    path.emplace_back(static_cast<double>(line_iterator.GetCurrentX()),
                      static_cast<double>(line_iterator.GetCurrentY()));
  }

  return path;
}

double LineConnector::getLinePathLength(const StateT& start,
                                        const StateT& goal) const {
  return std::hypot(goal.x - start.x, goal.y - start.y);
}

LineIteratorT LineConnector::createLineIterator(const StateT& start,
                                                const StateT& goal) const {
  return LineIteratorT{static_cast<int>(start.x), static_cast<int>(start.y),
                       static_cast<int>(goal.x), static_cast<int>(goal.y)};
}
