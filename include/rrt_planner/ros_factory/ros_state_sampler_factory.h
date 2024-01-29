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

#include "rrt_planner/param_loader/rgd_params_loader.h"
#include "rrt_planner/param_loader/sampling_params_loader.h"
#include "state_space/rgd_state_sampler/rgd_state_sampler.h"
#include "state_space/state_sampler/sampling_policy.h"
#include "state_space/state_sampler/state_sampler_params.h"
#include "state_space/state_sampler_factory/state_sampler_factory.h"

namespace rrt_planner::ros_factory {
class ROSStateSamplerFactory {
 public:
  using SamplingPolicyT = state_space::state_sampler::SamplingPolicy;
  template <typename StateT>
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  template <typename StateT>
  using StateSamplerPtr = std::shared_ptr<StateSamplerT<StateT>>;
  using CommonParamsT = state_space::state_sampler::StateSamplerParams;
  using StateSamplerFactoryT =
      state_space::state_sampler_factory::StateSamplerFactory;
  template <typename StateT>
  using RGDStateSamplerT =
      state_space::rgd_state_sampler::RGDStateSampler<StateT>;
  using RGDParamsT = state_space::rgd_state_sampler::RGDParams;

  ROSStateSamplerFactory() = delete;

  /**
   * @brief
   * @tparam StateT
   */
  template <typename StateT>
  static StateSamplerPtr<StateT> createSampler(SamplingPolicyT sampling_policy,
                                               unsigned int state_space_size,
                                               ros::NodeHandle* nh) {
    auto&& common_params =
        rrt_planner::param_loader::loadStateSamplerParams(nh, state_space_size);

    switch (sampling_policy) {
      case SamplingPolicyT::RGD_SAMPLING: {
        auto&& rgd_params = rrt_planner::param_loader::loadRGDParams(nh);
        return StateSamplerFactoryT::createRGDSampler<StateT>(
            std::move(common_params), std::move(rgd_params));
      }
      default:
        return nullptr;
    }
  }
};
}  // namespace rrt_planner::ros_factory

#endif
