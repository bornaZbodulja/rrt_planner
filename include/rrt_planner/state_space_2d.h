/**
 * @file state_space_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines 2D state space
 * @version 0.1
 * @date 2023-07-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cmath>

#ifndef RRT_PLANNER__STATE_SPACE_2D_H_
#define RRT_PLANNER__STATE_SPACE_2D_H_

namespace rrt_planner {

/**
 * @brief
 */
class StateSpace2D {
 public:
  /**
   * @brief State 2D representation
   */
  struct State {
    State() {}
    State(const double& x_in, const double& y_in) : x(x_in), y(y_in) {}

    double x{0.0}, y{0.0};
  };

  StateSpace2D() = default;

  /**
   * @brief State space initialization
   * @param size_x
   * @param size_y
   * @param dim_3
   */
  void InitializeStateSpace(const unsigned int& size_x,
                            const unsigned int& size_y,
                            const unsigned int& dim_3) {
    size_x_ = size_x;
    size_y_ = size_y;
    dim_3_ = 1;
  }

  /**
   * @brief Gets total number of possible states in state space
   * @return unsigned int
   */
  unsigned int GetStateSpaceSize() const { return size_x_ * size_y_ * dim_3_; }

  /**
   * @brief Computes index based on state space coordinates
   * @param x X position in space
   * @param y Y position in space
   * @return unsigned int
   */
  inline unsigned int GetIndex(const unsigned int& x,
                               const unsigned int& y) const {
    return y * size_x_ + x;
  }

  /**
   * @brief Computes index based on state
   * @param state
   * @return unsigned int
   */
  inline unsigned int GetIndex(const State& state) const {
    return GetIndex(static_cast<unsigned int>(state.x),
                    static_cast<unsigned int>(state.y));
  }

  /**
   * @brief Generates state space coordinates from state space index
   * @param index State space index
   * @return State
   */
  inline State GetState(const unsigned int& index) const {
    return State(index % size_x_, index / size_x_);
  }

  /**
   * @brief Computes distance between two states
   * @param start_state
   * @param goal_state
   * @return double
   */
  inline double GetStateSpaceDistance(const State& start_state,
                                      const State& goal_state) const {
    return std::hypot(goal_state.x - start_state.x,
                      goal_state.y - start_state.y);
  }

  /**
   * @brief Computes squared distance between two states
   * @param start_state
   * @param goal_state
   * @return double
   */
  inline double GetStateSpaceDistanceSquared(const State& start_state,
                                             const State& goal_state) const {
    return std::pow(goal_state.x - start_state.x, 2) +
           std::pow(goal_state.y - start_state.y, 2);
  }

 private:
  unsigned int size_x_{0}, size_y_{0}, dim_3_{0};
};
}  // namespace rrt_planner

#endif
