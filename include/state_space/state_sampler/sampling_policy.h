/**
 * @file sampling_policy.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines possible sampling policies
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SAMPLER__SAMPLING_POLICY_H_
#define STATE_SPACE__STATE_SAMPLER__SAMPLING_POLICY_H_

#include <string>

namespace state_space::state_sampler {
/**
 * @brief Representation of possible sampling policies
 */
enum class SamplingPolicy { UNKNOWN = 0, BASIC_SAMPLING = 1, RGD_SAMPLING = 2 };

inline std::string samplingPolicyToString(SamplingPolicy sampling_policy) {
  switch (sampling_policy) {
    case SamplingPolicy::RGD_SAMPLING:
      return "RGD_SAMPLING";
    case SamplingPolicy::BASIC_SAMPLING:
      return "BASIC_SAMPLING";
    default:
      return "Unknown";
  }
}

inline SamplingPolicy samplingPolicyFromString(std::string sampling_policy) {
  if (sampling_policy == "RGD_SAMPLING") {
    return SamplingPolicy::RGD_SAMPLING;
  } else if (sampling_policy == "BASIC_SAMPLING") {
    return SamplingPolicy::BASIC_SAMPLING;
  } else {
    return SamplingPolicy::UNKNOWN;
  }
}
}  // namespace state_space::state_sampler

#endif
