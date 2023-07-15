/**
 * @file state_space_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines hybrid state space
 * @version 0.1
 * @date 2023-07-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cmath>

#ifndef RRT_PLANNER__STATE_SPACE_HYBRID_H_
#define RRT_PLANNER__STATE_SPACE_HYBRID_H_

namespace rrt_planner {

/**
 * @brief
 */
class StateSpaceHybrid {
 public:
  /**
   * @brief State Hybrid representation
   */
  struct State {
    State() {}
    State(const double& x_in, const double& y_in, const double& theta_in)
        : x(x_in), y(y_in), theta(theta_in) {}

    double x{0.0}, y{0.0}, theta{0.0};
  };

  StateSpaceHybrid() = default;

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
    dim_3_ = dim_3;
    angle_bin = 2 * M_PI / dim_3_;
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
   * @param angle_idx Index of angle bin
   * @return unsigned int
   */
  inline unsigned int GetIndex(const unsigned int& x, const unsigned int& y,
                               const unsigned int& angle_idx) const {
    return angle_idx + x * dim_3_ + y * size_x_ * dim_3_;
  }

  /**
   * @brief Computes index based on state
   * @param state
   * @return unsigned int
   */
  inline unsigned int GetIndex(const State& state) {
    return GetIndex(static_cast<unsigned int>(state.x),
                    static_cast<unsigned int>(state.y),
                    static_cast<unsigned int>(state.theta));
  }

  /**
   * @brief Generates state space coordinates from state space index
   * @param index
   * @return State
   */
  inline State GetState(const unsigned int& index) const {
    return State((index / dim_3_) % size_x_, index / (dim_3_ * size_x_),
                       index & dim_3_);
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
                      goal_state.y - start_state.y) +
           std::abs(goal_state.theta - start_state.theta);
  }

  /**
   * @brief Computes squared distance between two states
   * @param start_state
   * @param goal_state
   * @return double
   */
  inline double GetStateSpaceDistanceSquared(
      const State& start_state, const State& goal_state) const {
    return std::pow(goal_state.x - start_state.x, 2) +
           std::pow(goal_state.y - start_state.y, 2);
  }

  /**
   * @brief Gets angular bin index
   * @param theta Raw orientation
   * @return unsigned int Index of bin
   */
  inline unsigned int GetClosestAngularBin(const double& theta) const {
    return static_cast<unsigned int>(std::floor(theta / angle_bin));
  }

  /**
   * @brief Gets raw orientation from bin
   * @param bin_idx Bin index
   * @return double
   */
  inline double GetAngleFromBin(const unsigned int& bin_idx) {
    return bin_idx * angle_bin;
  }

 private:
  unsigned int size_x_{0}, size_y_{0}, dim_3_{0};
  double angle_bin{0.0};
};

}  // namespace rrt_planner

#endif
