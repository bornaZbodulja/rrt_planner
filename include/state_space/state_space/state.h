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

namespace state_space {
/**
 * @brief State interface for state in state space
 * @tparam T
 */
template <typename T>
struct State {
  virtual ~State() = default;
  virtual void operator*(double k) = 0;
  virtual T operator+(const T& rhs) const = 0;
  virtual double norm() const = 0;

  inline virtual void operator/(double k) final { this->operator*(1 / k); };

  inline virtual T operator-(const T& rhs) const final {
    T res = rhs;
    res.operator*(-1.0);
    res = this->operator+(res);
    return res;
  }
};

}  // namespace state_space

#endif
