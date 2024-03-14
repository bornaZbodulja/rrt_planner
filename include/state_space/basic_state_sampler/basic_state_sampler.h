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

#include <experimental/random>
#include <memory>

#include "state_space/basic_state_sampler/basic_state_sampler_params.h"
#include "state_space/state_sampler/state_sampler.h"

namespace state_space::basic_state_sampler {
template <typename StateT>
class BasicStateSampler
    : public state_space::state_sampler::StateSampler<StateT> {
 public:
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;

  BasicStateSampler(BasicStateSamplerParams&& basic_state_sampler_params,
                    const StateSpacePtr& state_space)
      : StateSamplerT(),
        basic_state_sampler_params_(std::move(basic_state_sampler_params)),
        state_space_(state_space),
        gen_(std::minstd_rand(std::random_device{}())),
        dist_(std::uniform_real_distribution<double>(0.0, 1.0)) {}

  ~BasicStateSampler() override = default;

  unsigned int generateTreeExpansionIndex(unsigned int target_index) override {
    double r = dist_(gen_);

    if (r <= basic_state_sampler_params_.target_bias) {
      return target_index;
    }

    return generateRandomIndexInStateSpace();
  }

 protected:
  /**
   * @brief
   * @return unsigned int
   */
  unsigned int generateRandomIndexInStateSpace() const {
    constexpr unsigned int min_index = 0;
    return std::experimental::randint(min_index,
                                      state_space_->getStateSpaceSize());
  }

  // Sampler parameters
  BasicStateSamplerParams basic_state_sampler_params_;
  // State space pointer
  StateSpacePtr state_space_;
  // Utils for random numer generation
  std::minstd_rand gen_;
  std::uniform_real_distribution<double> dist_;
};
}  // namespace state_space::basic_state_sampler

#endif  // STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_H_
