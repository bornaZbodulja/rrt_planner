/**
 * @file basic_state_sampler_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic state sampler parameters
 * @version 0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_PARAMS_H_
#define STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_PARAMS_H_

namespace state_space::basic_state_sampler {
/**
 * @brief Representation of basic state sampler parameters
 */
struct BasicStateSamplerParams {
  BasicStateSamplerParams() = default;
  BasicStateSamplerParams(double target_bias_in)
      : target_bias(target_bias_in) {}

  // Defines bias towards target of the search tree when generating random node
  // (between 0.0 and 1.0)
  double target_bias{0.0};
  // State space size
  unsigned int state_space_size{0};
};
}  // namespace state_space::basic_state_sampler

#endif  // STATE_SPACE__BASIC_STATE_SAMPLER__BASIC_STATE_SAMPLER_PARAMS_H_
