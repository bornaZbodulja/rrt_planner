/**
 * @file state_sampler_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State sampler factory
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SAMPLER_FACTORY__STATE_SAMPLER_FACTORY_H_
#define STATE_SPACE__STATE_SAMPLER_FACTORY__STATE_SAMPLER_FACTORY_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "state_space/basic_state_sampler/basic_state_sampler.h"
#include "state_space/basic_state_sampler/basic_state_sampler_params.h"
#include "state_space/rgd_state_sampler/rgd_params.h"
#include "state_space/rgd_state_sampler/rgd_state_sampler.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace state_space::state_sampler_factory {
template <typename StateT>
inline std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
createBasicStateSampler(
    state_space::basic_state_sampler::BasicStateSamplerParams&&
        basic_state_sampler_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space) {
  return std::make_unique<
      state_space::basic_state_sampler::BasicStateSampler<StateT>>(
      std::move(basic_state_sampler_params), state_space);
}

template <typename StateT>
inline std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
createRGDStateSampler(
    state_space::basic_state_sampler::BasicStateSamplerParams&&
        basic_state_sampler_params,
    state_space::rgd_state_sampler::RGDParams&& rgd_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  return std::make_unique<
      state_space::rgd_state_sampler::RGDStateSampler<StateT>>(
      std::move(basic_state_sampler_params), std::move(rgd_params), state_space,
      collision_checker);
}
}  // namespace state_space::state_sampler_factory

#endif  // STATE_SPACE__STATE_SAMPLER_FACTORY__STATE_SAMPLER_FACTORY_H_
