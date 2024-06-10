/**
 * @file state_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State 2D definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_2D__STATE_2D_H_
#define STATE_SPACE__STATE_SPACE_2D__STATE_2D_H_

#include <cmath>
#include <vector>

#include "state_space/state_space/state.h"

namespace state_space::state_space_2d {
/**
 * @brief Representation of state in 2D state space
 */
class State2D : public State<State2D> {
 public:
  State2D() = default;
  explicit State2D(double x_in, double y_in) : x(x_in), y(y_in) {}
  explicit State2D(const std::vector<double>& state) {
    x = state.at(0);
    y = state.at(1);
  }

  bool operator==(const State2D& rhs) const { return x == rhs.x && y == rhs.y; }

  void operator*(double k) override {
    x *= k;
    y *= k;
  }

  State2D operator+(const State2D& rhs) const override {
    return State2D{x + rhs.x, y + rhs.y};
  }

  double squaredL2norm() const override {
    return std::pow(x, 2) + std::pow(y, 2);
  }

  double x{0.0}, y{0.0};
};
}  // namespace state_space::state_space_2d

#endif  // STATE_SPACE__STATE_SPACE_2D__STATE_2D_H_
