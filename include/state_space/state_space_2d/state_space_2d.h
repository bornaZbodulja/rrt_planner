/**
 * @file state_space_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State space 2D definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE__STATE_SPACE_2D_H_
#define STATE_SPACE__STATE_SPACE__STATE_SPACE_2D_H_

#include <nav_utils/collision_checker.h>

#include <cmath>

#include "state_space/state_space/state_space.h"
#include "state_space/state_space_2d/space_2d.h"
#include "state_space/state_space_2d/state_2d.h"

namespace state_space::state_space_2d {
class StateSpace2D : public state_space::StateSpace<State2D> {
 public:
  explicit StateSpace2D(Space2D&& space_in) : space_(std::move(space_in)) {}
  explicit StateSpace2D(double size_x_in, double size_y_in)
      : space_{size_x_in, size_y_in} {}

  void normalizeState(State2D& state) const override {
    static const double size_x = space_.getBounds().at(0);
    static const double size_y = space_.getBounds().at(1);
    state.x = std::fmod(state.x, size_x);
    state.y = std::fmod(state.y, size_y);
  }

  State2D getStatesDifference(const State2D& start_state,
                              const State2D& goal_state) const override {
    return (goal_state - start_state);
  }

  double getStateCost(
      const State2D& state,
      const nav_utils::CollisionChecker* const collision_checker) const {
    return collision_checker->getCost(static_cast<unsigned int>(state.x),
                                      static_cast<unsigned int>(state.y));
  }

  std::vector<double> getBounds() const override { return space_.getBounds(); }

 private:
  Space2D space_;
};
}  // namespace state_space::state_space_2d

#endif  // STATE_SPACE__STATE_SPACE__STATE_SPACE_2D_H_
