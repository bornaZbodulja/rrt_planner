/**
 * @file sampling_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__BASIC_SAMPLER_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__BASIC_SAMPLER_PARAMS_LOADER_H_

#include <nav_utils/type_utils.h>
#include <ros/console.h>

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/basic_state_sampler/basic_state_sampler_params.h"

namespace rrt_planner::param_loader {

inline state_space::basic_state_sampler::BasicStateSamplerParams
loadBasicSamplerParams(ros::NodeHandle* nh) {
  state_space::basic_state_sampler::BasicStateSamplerParams
      basic_sampler_params;
  loadParam<double>(nh, "target_bias", basic_sampler_params.target_bias);

  if (!nav_utils::valueInRange(basic_sampler_params.target_bias, 0.0, 1.0)) {
    ROS_ERROR(
        "Parameter %s is not in range [0.0, 1.0], terminating the program!",
        (nh->getNamespace() + "/target_bias").c_str());
    exit(0);
  }

  return basic_sampler_params;
}
}  // namespace rrt_planner::param_loader

#endif  // RRT_PLANNER__PARAM_LOADER__BASIC_SAMPLER_PARAMS_LOADER_H_
