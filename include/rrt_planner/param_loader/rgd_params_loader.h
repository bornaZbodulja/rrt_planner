/**
 * @file rgd_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__RGD_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__RGD_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/rgd_state_sampler/rgd_params.h"

namespace rrt_planner::param_loader {

inline state_space::rgd_state_sampler::RGDParams loadRGDParams(
    ros::NodeHandle* nh) {
  state_space::rgd_state_sampler::RGDParams rgd_params;
  loadParam<double>(nh, "RGD/increment_step", rgd_params.increment_step);
  loadParam<unsigned char>(nh, "RGD/stop_cost", rgd_params.stop_cost);
  loadParam<int>(nh, "RGD/iterations", rgd_params.iterations);

  return rgd_params;
}

}  // namespace rrt_planner::param_loader

#endif
