/**
 * @file nearest_neighbor_star_expander_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines nearest neighbor* expander parameters
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_PARAMS_H_
#define RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_PARAMS_H_

namespace rrt_planner::planner_core::nearest_neighbor_star_expander {
/**
 * @brief Representation of nearest neighbor* expander parameters
 */
struct NearestNeighborStarExpanderParams {
  NearestNeighborStarExpanderParams() = default;
  NearestNeighborStarExpanderParams(double near_distance_in,
                                    bool rewire_tree_in)
      : near_distance(near_distance_in), rewire_tree(rewire_tree_in) {}

  // Defines neighborhood of the nodes in the search tree
  double near_distance{0.0};
  // Whether to rewire tree after every expansion
  bool rewire_tree{false};
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_star_expander

#endif
