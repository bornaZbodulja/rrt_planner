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
  void operator*(double k) { derived_cast()->operator*(k); }

  T operator+(const T& rhs) const { return derived_cast()->operator+(rhs); }

  double squaredL2norm() const { return derived_cast()->squaredL2norm(); }

  double l2norm() { return std::sqrt(squaredL2norm()); }

  void operator/(double k) { this->operator*(1 / k); }

  T operator-(const T& rhs) const {
    T res = rhs;
    res.operator*(-1.0);
    res = this->operator+(res);
    return res;
  }

 private:
  T* derived_cast() { return (static_cast<T*>(this)); }

  const T* derived_cast() const { return (static_cast<const T*>(this)); }
};

}  // namespace state_space

#endif
