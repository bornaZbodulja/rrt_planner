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
#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"

namespace rrt_planner::param_loader {

inline rrt_planner::planner_core::planner::SearchParams loadSearchParams(
    ros::NodeHandle* nh) {
  rrt_planner::planner_core::planner::SearchParams search_params;
  loadParam<int>(nh, "max_expansion_iterations",
                 search_params.max_expansion_iterations);
  loadParam<double>(nh, "max_planning_timeout",
                    search_params.max_planning_time);
  return search_params;
}
}  // namespace rrt_planner::param_loader

#endif  // RRT_PLANNER__PARAM_LOADER__SEARCH_PARAMS_LOADER_H_
