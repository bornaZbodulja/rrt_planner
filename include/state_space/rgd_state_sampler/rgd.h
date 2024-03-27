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

#include <nav_utils/collision_checker.h>

#include "state_space/rgd_state_sampler/rgd_params.h"
#include "state_space/state_space/state_space.h"

namespace state_space::rgd_state_sampler {
/**
 * @brief
 * @tparam StateT
 */
template <typename StateT>
class RGD {
 public:
  explicit RGD(RGDParams&& rgd_params) : rgd_params_(std::move(rgd_params)) {}

  ~RGD() = default;

  /**
   * @brief Gradient descent towards target index
   * @param random_index Randomly generated index in state space
   * @param target_index Index of target node
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   * @return unsigned int
   */
  unsigned int operator()(
      unsigned int random_index, unsigned int target_index,
      const state_space::StateSpace<StateT>* const state_space,
      const nav_utils::CollisionChecker* const collision_checker) {
    if (rgd_params_.iterations <= 0) {
      return random_index;
    }

    StateT rand_state = state_space->getState(random_index);
    StateT target_state = state_space->getState(target_index);

    StateT mid_state;

    for (int i = 0; i < rgd_params_.iterations; i++) {
      if (state_space->getStateCost(rand_state, collision_checker) >
          rgd_params_.stop_cost) {
        break;
      }

      mid_state = target_state - rand_state;
      mid_state.operator*(rgd_params_.increment_step / mid_state.l2norm());
      rand_state = rand_state + mid_state;
      state_space->normalizeState(rand_state);
    }

    return state_space->getIndex(rand_state);
  }

 private:
  // RGD parameters
  RGDParams rgd_params_;
};

}  // namespace state_space::rgd_state_sampler

#endif
