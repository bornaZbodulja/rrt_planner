/**
 * @file ros_state_sampler_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__ROS_FACTORY__ROS_STATE_SAMPLER_FACTORY_H_
#define RRT_PLANNER__ROS_FACTORY__ROS_STATE_SAMPLER_FACTORY_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "rrt_planner/param_loader/basic_sampler_params_loader.h"
#include "rrt_planner/param_loader/rgd_params_loader.h"
#include "state_space/basic_state_sampler/basic_state_sampler.h"
#include "state_space/basic_state_sampler/basic_state_sampler_params.h"
#include "state_space/rgd_state_sampler/rgd_state_sampler.h"
#include "state_space/state_sampler/sampling_policy.h"
#include "state_space/state_sampler_factory/state_sampler_factory.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::ros_factory {

template <typename StateT>
inline std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
createStateSampler(
    ros::NodeHandle* nh,
    state_space::state_sampler::SamplingPolicy sampling_policy,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  state_space::basic_state_sampler::BasicStateSamplerParams
      basic_sampler_params = param_loader::loadBasicSamplerParams(nh);

  switch (sampling_policy) {
    case state_space::state_sampler::SamplingPolicy::BASIC_SAMPLING:
      return state_space::state_sampler_factory::createBasicStateSampler(
          std::move(basic_sampler_params), state_space);
    case state_space::state_sampler::SamplingPolicy::RGD_SAMPLING: {
      state_space::rgd_state_sampler::RGDParams rgd_params =
          param_loader::loadRGDParams(nh);
      return state_space::state_sampler_factory::createRGDStateSampler(
          std::move(basic_sampler_params), std::move(rgd_params), state_space,
          collision_checker);
    }
    default:
      return nullptr;
  }
}
}  // namespace rrt_planner::ros_factory

#endif
