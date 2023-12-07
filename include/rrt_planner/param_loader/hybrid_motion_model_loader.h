/**
 * @file hybrid_motion_model_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__HYBRID_MOTION_MODEL_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__HYBRID_MOTION_MODEL_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/state_connector_hybrid/motion_model_hybrid.h"

namespace rrt_planner::param_loader {

inline state_space::state_connector_hybrid::HybridMotionModel
loadHybridMotionModel(ros::NodeHandle* nh) {
  std::string motion_model_str;
  loadParam<std::string>(nh, "motion_model", motion_model_str);
  auto motion_model =
      state_space::state_connector_hybrid::hybridMotionModelFromString(
          motion_model_str);

  if (motion_model ==
      state_space::state_connector_hybrid::HybridMotionModel::UNKNOWN) {
    ROS_ERROR(
        "Unknown hybrid motion model for planner: %s. Terminating the program.",
        motion_model_str.c_str());
    exit(0);
  }

  return motion_model;
}

}  // namespace rrt_planner::param_loader

#endif
