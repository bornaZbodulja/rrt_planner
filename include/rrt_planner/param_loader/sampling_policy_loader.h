/**
 * @file sampling_policy_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__SAMPLING_POLICY_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__SAMPLING_POLICY_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/state_sampler/sampling_policy.h"

namespace rrt_planner::param_loader {

inline state_space::state_sampler::SamplingPolicy loadSamplingPolicy(
    ros::NodeHandle* nh) {
  std::string sampling_policy_str;
  loadParam<std::string>(nh, "sampling_policy", sampling_policy_str);
  auto sampling_policy =
      state_space::state_sampler::samplingPolicyFromString(sampling_policy_str);

  if (sampling_policy == state_space::state_sampler::SamplingPolicy::UNKNOWN) {
    ROS_ERROR(
        "Unknown sampling policy for planner: %s. Terminating the program!",
        sampling_policy_str.c_str());
    exit(0);
  }

  return sampling_policy;
}
}  // namespace rrt_planner::param_loader

#endif // RRT_PLANNER__PARAM_LOADER__SAMPLING_POLICY_LOADER_H_
