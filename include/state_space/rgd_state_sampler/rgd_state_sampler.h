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
  using BasicStateSamplerT =
      state_space::basic_state_sampler::BasicStateSampler<StateT>;
  using BasicStateSamplerParamsT =
      state_space::basic_state_sampler::BasicStateSamplerParams;
  using StateSpacePtr = typename BasicStateSamplerT::StateSpacePtr;
  using CollisionCheckerPtr = std::shared_ptr<nav_utils::CollisionChecker>;

  /**
   * @brief RGD state sampler constructor
   * @param sampler_params Basic state sampler parameters
   * @param rgd_params RGD parameters
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   */
  RGDStateSampler(BasicStateSamplerParamsT&& basic_state_sampler_params,
                  RGDParams&& rgd_params, const StateSpacePtr& state_space,
                  const CollisionCheckerPtr& collision_checker)
      : BasicStateSamplerT(std::move(basic_state_sampler_params), state_space),
        rgd_params_(std::move(rgd_params)),
        collision_checker_(collision_checker) {}

  ~RGDStateSampler() override = default;

  /**
   * @brief Generates new index for search tree expansion
   * @param target_index Index of target node of the search tree
   * @return unsigned int
   */
  unsigned int generateTreeExpansionIndex(unsigned int target_index) override {
    unsigned int new_index =
        BasicStateSamplerT::generateTreeExpansionIndex(target_index);

    if (new_index == target_index) {
      return new_index;
    }

    return rgd_.gradientDescent(
        new_index, target_index, this->state_space_.get(),
        rgd_params_.increment_step, rgd_params_.stop_cost,
        rgd_params_.iterations, this->collision_checker_.get());
  }

 protected:
  // RGD parameters
  RGDParams rgd_params_;
  // Random gradient descent
  RGD<StateT> rgd_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
};

}  // namespace state_space::rgd_state_sampler

#endif