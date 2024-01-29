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

#include <memory>

#include "state_space/rgd_state_sampler/rgd_params.h"
#include "state_space/rgd_state_sampler/rgd_state_sampler.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_sampler/state_sampler_params.h"

namespace state_space::state_sampler_factory {
class StateSamplerFactory {
 public:
  template <typename StateT>
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  template <typename StateT>
  using StateSamplerPtr = std::shared_ptr<StateSamplerT<StateT>>;
  using CommonParamsT = state_space::state_sampler::StateSamplerParams;
  template <typename StateT>
  using RGDStateSamplerT =
      state_space::rgd_state_sampler::RGDStateSampler<StateT>;
  using RGDParamsT = state_space::rgd_state_sampler::RGDParams;

  StateSamplerFactory() = delete;

  /**
   * @brief
   * @tparam StateT
   * @param common_params
   * @param rgd_params
   * @return StateSamplerPtr<StateT>
   */
  template <typename StateT>
  static StateSamplerPtr<StateT> createRGDSampler(CommonParamsT&& common_params,
                                                  RGDParamsT&& rgd_params) {
    return std::make_shared<RGDStateSamplerT<StateT>>(std::move(common_params),
                                                      std::move(rgd_params));
  }
};
}  // namespace state_space::state_sampler_factory

#endif
