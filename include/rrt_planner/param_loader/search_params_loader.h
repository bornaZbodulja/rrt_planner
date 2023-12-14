/**
 * @file search_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__SEARCH_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__SEARCH_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "rrt_planner/planner_core/search_params.h"
#include "rrt_planner/planner_core/search_policy.h"

namespace rrt_planner::param_loader {

/**
 * @brief
 * @param nh
 * @param search_policy
 * @param costmap_resolution
 * @return rrt_planner::planner_core::SearchParams
 */
inline rrt_planner::planner_core::SearchParams loadSearchParams(
    ros::NodeHandle* nh, rrt_planner::planner_core::SearchPolicy search_policy,
    double costmap_resolution) {
  rrt_planner::planner_core::SearchParams search_params;
  loadParam<double>(nh, "cost_penalty", search_params.cost_penalty);
  loadParam<double>(nh, "traversal_cost", search_params.traversal_cost);
  loadParam<int>(nh, "max_expansion_iterations",
                 search_params.max_expansion_iterations);
  loadParam<double>(nh, "max_planning_timeout",
                    search_params.max_planning_time);

  if (search_policy == rrt_planner::planner_core::SearchPolicy::RRT_STAR ||
      search_policy ==
          rrt_planner::planner_core::SearchPolicy::BIDIRECTIONAL_RRT_STAR) {
    loadParam<bool>(nh, "rewire_tree", search_params.rewire_tree);
    loadParam<double>(nh, "near_distance", search_params.near_distance);
    // Scaling with costmap resolution
    search_params.near_distance /= costmap_resolution;
  }

  if (search_policy ==
          rrt_planner::planner_core::SearchPolicy::BIDIRECTIONAL_RRT ||
      search_policy ==
          rrt_planner::planner_core::SearchPolicy::BIDIRECTIONAL_RRT_STAR) {
    loadParam<double>(nh, "tree_connection_max_length",
                      search_params.tree_connection_max_length);
    // Scaling with costmap resolution
    search_params.tree_connection_max_length /= costmap_resolution;
  }

  return search_params;
}

}  // namespace rrt_planner::param_loader

#endif
