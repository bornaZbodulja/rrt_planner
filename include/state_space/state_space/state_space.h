/**
 * @file state_space.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State space interface
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE__STATE_SPACE_H_
#define STATE_SPACE__STATE_SPACE__STATE_SPACE_H_

#include <nav_utils/collision_checker.h>

#include "state_space/state_space/space.h"

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
   * @brief Normalizes given state
   * @param state
   */
  virtual void normalizeState(StateT& state) const = 0;

  /**
   * @brief Computes difference between two given states
   * @param start_state
   * @param goal_state
   * @return double
   */
  virtual StateT getStatesDifference(const StateT& start_state,
                                     const StateT& goal_state) const = 0;

  /**
   * @brief Computes cost of given state
   * @param state
   * @return double
   */
  virtual double getStateCost(
      const StateT& state,
      const nav_utils::CollisionChecker* const collision_checker) const = 0;

  /**
   * @brief Gets space bounds with state space
   * @return const Space&
   */
  virtual std::vector<double> getBounds() const = 0;
};
}  // namespace state_space

#endif
