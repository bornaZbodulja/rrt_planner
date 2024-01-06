/**
 * @file rgd.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Random gradient descent implementation
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__RGD_STATE_SAMPLER__RGD_H_
#define STATE_SPACE__RGD_STATE_SAMPLER__RGD_H_

#include <nav_utils/nav_utils.h>

#include "state_space/state_space/state_space.h"

namespace state_space::rgd_state_sampler {
/**
 * @brief
 * @tparam StateT
 */
template <typename StateT>
struct RGD {
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = StateSpaceT*;

  RGD() = default;
  ~RGD() = default;

  /**
   * @brief Gradient descent towards target node
   * @param random_index Randomly generated index in state space
   * @param target_index Index of target node
   * @param state_space State space pointer
   * @param increment_step Increment step of the descent
   * @param stop_cost Stop cost for the descent
   * @param iterations Iterations of the descent
   * @param collision_checker Collision checker pointer
   * @return unsigned int
   */
  unsigned int gradientDescent(unsigned int random_index,
                               unsigned int target_index,
                               StateSpacePtr state_space, double increment_step,
                               unsigned char stop_cost, int iterations,
                               const CollisionCheckerPtr& collision_checker) {
    if (iterations <= 0) {
      return random_index;
    }

    auto rand_state = state_space->getState(random_index);
    auto target_state = state_space->getState(target_index);

    StateT mid_state;

    for (int i = 0; i < iterations; i++) {
      if (collision_checker->getCost(static_cast<unsigned int>(rand_state.x),
                                     static_cast<unsigned int>(rand_state.y)) >
          stop_cost) {
        break;
      }

      mid_state = target_state - rand_state;
      mid_state.operator*(increment_step / mid_state.l2norm());
      rand_state = rand_state + mid_state;
      state_space->normalizeState(rand_state);
    }

    return state_space->getIndex(rand_state);
  }
};

}  // namespace state_space::rgd_state_sampler

#endif
