/**
 * @file search_info.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planning parameters definition
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/motion_model.h"

#ifndef RRT_PLANNER__SEARCH_INFO_H_
#define RRT_PLANNER__SEARCH_INFO_H_

namespace rrt_planner {

/**
 * @brief Holds search properties and penalties for planning
 */
struct SearchInfo {
  // Defines length of the edges connecting nodes in search tree in map cells
  int edge_length{0};
  // Defines bias towards target of the search tree when generating random node
  // (between 0.0 and 1.0)
  double target_bias{0.0};
  // Defines neighborhood of the nodes in the search tree
  double near_distance{0.0};
  // Penalty to apply to high cost areas
  double cost_penalty{0.0};
  // Minimal turning radius of the vehicle
  double min_turning_radius{0.0};
  // Whether to rewire tree after every expansion
  bool rewire_tree{false};
  // Max length of connection between trees
  double connect_trees_max_length{0.0};
  // Increment step for random gradient descent (between 0.0 and 1.0)
  double rgd_increment_step{0.0};
  // Stop cost random gradient descent
  unsigned char rgd_stop_cost{0};
  // Number of iterations for random gradient descent
  int rgd_iterations{0};
  // Whether to allow expansion in unknown space
  bool allow_unknown{false};
  // Lethal cost for collision checking
  unsigned char lethal_cost{0};
  // Maximum number of search tree expansions
  int max_expansion_iterations{0};
  // Maximum allowed planning time
  double max_planning_time{0.0};
  // Motion model
  MotionModel motion_model{MotionModel::UNKNOWN};
};

}  // namespace rrt_planner

#endif
