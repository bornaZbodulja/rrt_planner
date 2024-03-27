/**
 * @file search_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines parameters for planning
 * @version 0.1
 * @date 2023-09-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__SEARCH_PARAMS_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__SEARCH_PARAMS_H_

namespace rrt_planner::planner_core::planner_implementations {
/**
 * @brief Representation of planning search parameters
 */
struct SearchParams {
  // Maximum number of search tree expansions
  int max_expansion_iterations{0};
  // Maximum allowed planning time
  double max_planning_time{0.0};
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__SEARCH_PARAMS_H_
