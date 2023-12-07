/**
 * @file hybrid_model_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__HYBRID_MODEL_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__HYBRID_MODEL_LOADER_H_

#include "rrt_planner/param_loader/hybrid_motion_model_loader.h"
#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/state_connector_hybrid/model_hybrid.h"

namespace rrt_planner::param_loader {

/**
 * @brief 
 * @param nh 
 * @param costmap_resolution 
 * @return state_space::state_connector_hybrid::HybridModel 
 */
inline state_space::state_connector_hybrid::HybridModel loadHybridModel(
    ros::NodeHandle* nh, double costmap_resolution) {
  state_space::state_connector_hybrid::HybridModel hybrid_model;
  hybrid_model.hybrid_motion_model = loadHybridMotionModel(nh);
  loadParam<double>(nh, "min_turning_radius", hybrid_model.min_turning_radius);
  // Scaling with costmap resolution
  hybrid_model.min_turning_radius /= costmap_resolution;
  return hybrid_model;
}

}  // namespace rrt_planner::param_loader

#endif
