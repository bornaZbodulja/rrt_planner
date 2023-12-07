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

#ifndef RRT_PLANNER__PARAM_LOADER__SAMPLING_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__SAMPLING_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/state_sampler/state_sampler_params.h"

namespace rrt_planner::param_loader {

inline state_space::state_sampler::StateSamplerParams loadStateSamplerParams(
    ros::NodeHandle* nh, unsigned int state_space_size) {
  state_space::state_sampler::StateSamplerParams sampler_params;
  loadParam<double>(nh, "target_bias", sampler_params.target_bias);
  sampler_params.state_space_size = state_space_size;
  return sampler_params;
}

}  // namespace rrt_planner::param_loader

#endif
