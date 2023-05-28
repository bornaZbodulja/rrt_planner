/**
 * @file types.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__TYPES_H_
#define RRT_PLANNER__TYPES_H_

namespace rrt_planner {

/**
 * @brief Holds search properties and penalties for planning
 */
struct SearchInfo {
  // Defines length of the edges connecting nodes in search tree
  int edge_length{0};
  // Defines bias towards target of the search tree when generating random node
  // (between 0.0 and 1.0)
  double target_bias{0.0};
  // Defines neighborhood of the nodes in the search tree
  double near_distance{0.0};
  // Penalty to apply to high cost areas
  double cost_penalty{0.0};
  // Whether to rewire tree after every expansion
  bool rewire_tree{false};
  // Whether to allow expansion in unknown space
  bool allow_unknown{false};
  // Lethal cost for collision checking
  unsigned char lethal_cost{0};
  // Maximum number of search tree expansions
  int max_expansion_iterations{0};
  // Maximum allowed planning time
  double max_planning_time{0.0};
};

}  // namespace rrt_planner

#endif
