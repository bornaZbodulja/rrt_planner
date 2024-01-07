/**
 * @file rgd_state_sampler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RGD state sampler
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__RGD_STATE_SAMPLER__RGD_STATE_SAMPLER_H_
#define STATE_SPACE__RGD_STATE_SAMPLER__RGD_STATE_SAMPLER_H_

#include <nav_utils/nav_utils.h>

#include <experimental/random>

#include "state_space/rgd_state_sampler/rgd.h"
#include "state_space/rgd_state_sampler/rgd_params.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_sampler/state_sampler_params.h"
#include "state_space/state_space/state_space.h"

namespace state_space::rgd_state_sampler {
/**
 * @brief
 * @tparam StateT
 */
template <typename StateT>
class RGDStateSampler
    : public state_space::state_sampler::StateSampler<StateT> {
 public:
  using StateSamplerBaseT = state_space::state_sampler::StateSampler<StateT>;
  using StateSamplerParamsT = state_space::state_sampler::StateSamplerParams;
  using StateSamplerBaseT::params_;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = StateSpaceT*;

  explicit RGDStateSampler(StateSamplerParamsT&& sampler_params,
                           RGDParams&& rgd_params)
      : StateSamplerBaseT(std::move(sampler_params)),
        rgd_params_(std::move(rgd_params)),
        gen_(std::minstd_rand(std::random_device{}())),
        dist_(std::uniform_real_distribution<double>(0.0, 1.0)) {}

  ~RGDStateSampler() override = default;

  /**
   * @brief Generates new index for search tree expansion
   * @param target_index Index of target node of the search tree
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   * @return unsigned int
   */
  unsigned int generateTreeExpansionIndex(
      unsigned int target_index, StateSpacePtr state_space,
      const CollisionCheckerPtr& collision_checker) override {
    const auto r = dist_(gen_);

    if (r <= params_.target_bias) {
      return target_index;
    }

    return rgd_.gradientDescent(generateRandomIndexInStateSpace(), target_index,
                                state_space, rgd_params_.increment_step,
                                rgd_params_.stop_cost, rgd_params_.iterations,
                                collision_checker);
  }

 protected:
  unsigned int generateRandomIndexInStateSpace() const {
    return std::experimental::randint(static_cast<unsigned int>(0),
                                      params_.state_space_size);
  }

  // RGD parameters
  RGDParams rgd_params_;
  // Random gradient descent
  RGD<StateT> rgd_;
  // Utils for random numer generation
  std::minstd_rand gen_;
  std::uniform_real_distribution<double> dist_;
};

}  // namespace state_space::rgd_state_sampler

#endif