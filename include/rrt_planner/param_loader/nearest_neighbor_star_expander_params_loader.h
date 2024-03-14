/**
 * @file nearest_neighbor_star_expander_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_STAR_EXPANDER_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_STAR_EXPANDER_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander_params.h"

namespace rrt_planner::param_loader {

inline rrt_planner::planner_core::nearest_neighbor_star_expander::
    NearestNeighborStarExpanderParams
    loadNearestNeighborExpanderParams(ros::NodeHandle* nh,
                                      double map_resolution) {
  rrt_planner::planner_core::nearest_neighbor_star_expander::
      NearestNeighborStarExpanderParams nearest_neighbor_star_expander_params;
  loadParam<double>(nh, "near_distance",
                    nearest_neighbor_star_expander_params.near_distance);
  loadParam<bool>(nh, "rewire_tree",
                  nearest_neighbor_star_expander_params.rewire_tree);

  // Scaling with costmap resolution
  nearest_neighbor_star_expander_params.near_distance /= map_resolution;
  return nearest_neighbor_star_expander_params;
}
}  // namespace rrt_planner::param_loader

#endif  // RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_STAR_EXPANDER_PARAMS_LOADER_H_
