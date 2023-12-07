/**
 * @file rgd_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RGD params
 * @version 0.1
 * @date 2023-10-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__RGD_STATE_SAMPLER__RGD_PARAMS_H_
#define STATE_SPACE__RGD_STATE_SAMPLER__RGD_PARAMS_H_

namespace state_space::rgd_state_sampler {
/**
 * @brief Representation of random gradient descent parameters
 */
struct RGDParams {
  RGDParams() = default;
  RGDParams(double increment_step_in, unsigned char stop_cost_in,
            unsigned int iterations_in)
      : increment_step(increment_step_in),
        stop_cost(stop_cost_in),
        iterations(iterations_in) {}

  // Increment step for random gradient descent (between 0.0 and 1.0)
  double increment_step{0.0};
  // Stop cost random gradient descent
  unsigned char stop_cost{0};
  // Number of iterations for random gradient descent
  int iterations{0};
};
}  // namespace state_space::rgd_state_sampler

#endif
