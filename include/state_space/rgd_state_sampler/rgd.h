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
   * @brief Gradient descent towards target state
   * @param random_state Randomly generated state in state space
   * @param target_state State of target node
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   * @return StateT
   */
  StateT operator()(
      StateT random_state, StateT target_state,
      const state_space::StateSpace<StateT>* const state_space,
      const nav_utils::CollisionChecker* const collision_checker) {
    if (rgd_params_.iterations <= 0) {
      return random_state;
    }

    static StateT mid_state;

    for (int i = 0; i < rgd_params_.iterations; i++) {
      if (state_space->getStateCost(random_state, collision_checker) >
          rgd_params_.stop_cost) {
        break;
      }

      mid_state = target_state - random_state;
      mid_state.operator*(rgd_params_.increment_step / mid_state.l2norm());
      random_state = random_state + mid_state;
      state_space->normalizeState(random_state);
    }

    return random_state;
  }

 private:
  // RGD parameters
  RGDParams rgd_params_;
};

}  // namespace state_space::rgd_state_sampler

#endif
