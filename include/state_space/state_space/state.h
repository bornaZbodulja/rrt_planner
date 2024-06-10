/**
 * @file state.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State interface
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE__STATE_H_
#define STATE_SPACE__STATE_SPACE__STATE_H_

#include <cmath>

namespace state_space {
/**
 * @brief State interface for state in state space
 * @tparam T
 */
template <typename T>
class State {
 public:
  virtual void operator*(double k) = 0;

  virtual T operator+(const T& rhs) const = 0;

  virtual double squaredL2norm() const = 0;

  double l2norm() { return std::sqrt(squaredL2norm()); }

  void operator/(double k) { this->operator*(1 / k); }

  T operator-(const T& rhs) const {
    T res = rhs;
    res.operator*(-1.0);
    res = this->operator+(res);
    return res;
  }
};

}  // namespace state_space

#endif
