/**
 * @file basic_state_sampler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic state sampler implementation
 * @version 0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_H_
#define STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_H_

#include <algorithm>
#include <experimental/random>
#include <memory>
#include <vector>

#include "state_space/basic_state_sampler/basic_state_sampler_params.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/space.h"

namespace state_space::basic_state_sampler {
template <typename StateT>
class BasicStateSampler
    : public state_space::state_sampler::StateSampler<StateT> {
 public:
  explicit BasicStateSampler(
      BasicStateSamplerParams&& basic_state_sampler_params,
      const state_space::Space& space)
      : state_space::state_sampler::StateSampler<StateT>(),
        basic_state_sampler_params_(std::move(basic_state_sampler_params)),
        space_bounds_(space.getBounds()),
        gen_(std::minstd_rand(std::random_device{}())),
        dist_(std::uniform_real_distribution<double>(0.0, 1.0)) {}

  ~BasicStateSampler() override = default;

  StateT generateTreeExpansionState(StateT target_state) override {
    double r = generateRandomDouble(0.0, 1.0);

    if (r <= basic_state_sampler_params_.target_bias) {
      return target_state;
    }

    return generateRandomStateInStateSpace();
  }

 private:
  /**
   * @brief
   * @return double
   */
  StateT generateRandomStateInStateSpace() {
    std::vector<double> state_vector;
    state_vector.reserve(space_bounds_.size());

    std::transform(space_bounds_.begin(), space_bounds_.end(),
                   std::back_inserter(state_vector), [this](double bound) {
                     return generateRandomDouble(0.0, bound);
                   });
    StateT state;
    state = state_vector;
    return state;
  }

  double generateRandomDouble(double lower_bound, double upper_bound) {
    return lower_bound + dist_(gen_) * (upper_bound - lower_bound);
  }

  // Sampler parameters
  BasicStateSamplerParams basic_state_sampler_params_;
  // State space bounds
  std::vector<double> space_bounds_;
  // Utils for random numer generation
  std::minstd_rand gen_;
  std::uniform_real_distribution<double> dist_;
};
}  // namespace state_space::basic_state_sampler

#endif  // STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_H_
