/**
 * @file state_space.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic state space definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE__STATE_SPACE_H_
#define STATE_SPACE__STATE_SPACE__STATE_SPACE_H_

#include <nav_utils/collision_checker.h>

namespace state_space {
/**
 * @brief State space interface
 * @tparam StateT State template
 */
template <typename StateT>
class StateSpace {
 public:
  virtual ~StateSpace() = default;

  /**
   * @brief Computes index of given state in state space
   * @param state
   * @return unsigned int
   */
  virtual unsigned int getIndex(const StateT& state) const = 0;

  /**
   * @brief Generates state for given index
   * @param index
   * @return StateT
   */
  virtual StateT getState(unsigned int index) const = 0;

  /**
   * @brief Normalizes given state
   * @param state
   */
  virtual void normalizeState(StateT& state) const = 0;

  /**
   * @brief Computes distance between two given states
   * @param start_state
   * @param goal_state
   * @return double
   */
  virtual StateT getStateDistance(const StateT& start_state,
                                  const StateT& goal_state) const = 0;

  /**
   * @brief Computes size of state space
   * @return unsigned int
   */
  virtual unsigned int getStateSpaceSize() const = 0;

  /**
   * @brief Computes cost of given state
   * @param state
   * @return double
   */
  virtual double getStateCost(
      const StateT& state,
      const nav_utils::CollisionChecker* const collision_checker) const = 0;
};

}  // namespace state_space

#endif
