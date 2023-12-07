/**
 * @file state_sampler_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State sampler parameters
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_PARAMS_H_
#define STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_PARAMS_H_

namespace state_space::state_sampler {
/**
 * @brief Representation of state sampler parameters
 */
struct StateSamplerParams {
  StateSamplerParams() = default;
  StateSamplerParams(double target_bias_in, unsigned int state_space_size_in)
      : target_bias(target_bias_in), state_space_size(state_space_size_in) {}

  // Defines bias towards target of the search tree when generating random node
  // (between 0.0 and 1.0)
  double target_bias{0.0};
  unsigned int state_space_size{0};
};

}  // namespace state_space::state_sampler

#endif
