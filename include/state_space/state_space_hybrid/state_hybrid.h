/**
 * @file state_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State hybrid definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_HYBRID__STATE_HYBRID_H_
#define STATE_SPACE__STATE_SPACE_HYBRID__STATE_HYBRID_H_

#include <cmath>

#include "state_space/state_space/state.h"

namespace state_space::state_space_hybrid {
/**
 * @brief Representation of state in hybrid state space
 */
class StateHybrid : public State<StateHybrid> {
 public:
  StateHybrid() = default;
  explicit StateHybrid(double x_in, double y_in, double theta_in)
      : x(x_in), y(y_in), theta(theta_in) {}

  bool operator==(const StateHybrid& rhs) const {
    return x == rhs.x && y == rhs.y && theta == rhs.theta;
  }

  void operator*(double k) {
    x *= k;
    y *= k;
    theta *= k;
  }

  StateHybrid operator+(const StateHybrid& rhs) const {
    return StateHybrid{x + rhs.x, y + rhs.y, theta + rhs.theta};
  }

  double squaredL2norm() const {
    double d = std::pow(x, 2) + std::pow(y, 2);
    constexpr double kSwitchThreshold = 1.0;

    // TODO: Implement this to be continuous function
    if (d > kSwitchThreshold) {
      return d + (1 / d) * std::abs(theta);
    } else {
      return d + std::abs(theta);
    }
  }

  StateHybrid& operator=(const std::vector<double>& state) {
    x = state[0];
    y = state[1];
    theta = state[2];
    return *this;
  }

  double x{0.0}, y{0.0}, theta{0.0};
};
}  // namespace state_space::state_space_hybrid

#endif  // STATE_SPACE__STATE_SPACE_HYBRID__STATE_HYBRID_H_
