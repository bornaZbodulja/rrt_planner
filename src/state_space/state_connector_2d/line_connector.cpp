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

std::optional<LineConnector::State2D> LineConnector::tryLineExpand(
    const State2D& start, const State2D& target) const {
  nav_utils::LineIterator line_iterator = createLineIterator(start, target);
  // skip start position
  line_iterator.advance();

  // TODO: Simplify this
  int iteration{0};

  while (line_iterator.isValid() &&
         iteration < connector_params_.max_extension_states) {
    if (pointInCollision(line_iterator.getCurrentX(),
                         line_iterator.getCurrentY())) {
      return std::nullopt;
    }
    iteration++;
    line_iterator.advance();
  }

  return std::make_optional<State2D>(
      static_cast<double>(line_iterator.getPreviousX()),
      static_cast<double>(line_iterator.getPreviousY()));
}

bool LineConnector::tryConnectStates(const State2D& start,
                                     const State2D& goal) const {
  nav_utils::LineIterator line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.advance();

  // TODO: Simplify this

  while (line_iterator.isValid()) {
    if (pointInCollision(line_iterator.getCurrentX(),
                         line_iterator.getCurrentY())) {
      return false;
    }
    line_iterator.advance();
  }

  return true;
}

std::vector<LineConnector::State2D> LineConnector::getLinePath(
    const State2D& start, const State2D& goal) const {
  std::vector<State2D> path;

  nav_utils::LineIterator line_iterator = createLineIterator(start, goal);
  // skip start position
  line_iterator.advance();

  while (line_iterator.isValid()) {
    path.emplace_back(static_cast<double>(line_iterator.getCurrentX()),
                      static_cast<double>(line_iterator.getCurrentY()));
    line_iterator.advance();
  }

  return path;
}

double LineConnector::getLinePathLength(const State2D& start,
                                        const State2D& goal) const {
  return std::hypot(goal.x - start.x, goal.y - start.y);
}

nav_utils::LineIterator LineConnector::createLineIterator(
    const State2D& start, const State2D& goal) const {
  return nav_utils::LineIterator{
      static_cast<int>(start.x), static_cast<int>(start.y),
      static_cast<int>(goal.x), static_cast<int>(goal.y)};
}

bool LineConnector::pointInCollision(unsigned int x, unsigned int y) const {
  return collision_checker_->pointInCollision(
      x, y, connector_params_.lethal_cost, connector_params_.allow_unknown);
}
}  // namespace state_space::state_connector_2d
