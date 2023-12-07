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

#ifndef RRT_PLANNER__PLANNER_CORE__SEARCH_PARAMS_H_
#define RRT_PLANNER__PLANNER_CORE__SEARCH_PARAMS_H_

#include "rrt_planner/planner_core/search_policy.h"

namespace rrt_planner::planner_core {
/**
 * @brief Representation of planning search parameters
 */
struct SearchParams {
  // Defines neighborhood of the nodes in the search tree
  double near_distance{0.0};
  // Penalty to apply to high cost areas
  double cost_penalty{0.0};
  // Penalty to apply to traversal distance between parent and child node
  double traversal_cost{0.0};
  // Whether to rewire tree after every expansion
  bool rewire_tree{false};
  // Max length of connection between trees
  double tree_connection_max_length{0.0};
  // Maximum number of search tree expansions
  int max_expansion_iterations{0};
  // Maximum allowed planning time
  double max_planning_time{0.0};
};

}  // namespace rrt_planner::planner_core

#endif
