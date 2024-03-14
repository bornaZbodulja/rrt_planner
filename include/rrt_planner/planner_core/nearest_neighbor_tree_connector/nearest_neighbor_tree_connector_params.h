/**
 * @file nearest_neighbor_tree_connector_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines nearest neighbor tree connector parameters
 * @version 0.1
 * @date 2024-02-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_H_
#define RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_H_

namespace rrt_planner::planner_core::nearest_neighbor_tree_connector {
struct NearestNeighborTreeConnectorParams {
  NearestNeighborTreeConnectorParams() = default;
  NearestNeighborTreeConnectorParams(double tree_connection_max_length_in)
      : tree_connection_max_length(tree_connection_max_length_in) {}

  // Max length of connection between trees
  double tree_connection_max_length{0.0};
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_tree_connector

#endif  // RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_H_
