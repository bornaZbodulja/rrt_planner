/**
 * @file rgd_state_sampler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RGD state sampler implementation
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__RGD_STATE_SAMPLER__RGD_STATE_SAMPLER_H_
#define STATE_SPACE__RGD_STATE_SAMPLER__RGD_STATE_SAMPLER_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "state_space/basic_state_sampler/basic_state_sampler.h"
#include "state_space/basic_state_sampler/basic_state_sampler_params.h"
#include "state_space/rgd_state_sampler/rgd.h"
#include "state_space/rgd_state_sampler/rgd_params.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/space.h"
#include "state_space/state_space/state_space.h"

namespace state_space::rgd_state_sampler {
/**
 * @brief
 * @tparam StateT
 */
template <typename StateT>
class RGDStateSampler
    : public state_space::basic_state_sampler::BasicStateSampler<StateT> {
 public:
  /**
   * @brief RGD state sampler constructor
   * @param sampler_params Basic state sampler parameters
   * @param rgd_params RGD parameters
   * @param space Space reference
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   */
  explicit RGDStateSampler(
      state_space::basic_state_sampler::BasicStateSamplerParams&&
          basic_state_sampler_params,
      RGDParams&& rgd_params,
      const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
      const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker)
      : state_space::basic_state_sampler::BasicStateSampler<StateT>(
            std::move(basic_state_sampler_params), state_space->getBounds()),
        rgd_{std::move(rgd_params)},
        state_space_(state_space),
        collision_checker_(collision_checker) {}

  ~RGDStateSampler() override = default;

  /**
   * @brief Generates new state for search tree expansion
   * @param target_state State of target node of the search tree
   * @return StateT
   */
  StateT generateTreeExpansionState(StateT target_state) override {
    StateT new_state = state_space::basic_state_sampler::BasicStateSampler<
        StateT>::generateTreeExpansionState(target_state);

    if (new_state == target_state) {
      return target_state;
    }

    return (rgd_)(new_state, target_state, state_space_.get(),
                  collision_checker_.get());
  }

 private:
  // Random gradient descent
  RGD<StateT> rgd_;
  // State space pointer
  std::shared_ptr<state_space::StateSpace<StateT>> state_space_;
  // Collision checker pointer
  std::shared_ptr<nav_utils::CollisionChecker> collision_checker_;
};

}  // namespace state_space::rgd_state_sampler

#endif