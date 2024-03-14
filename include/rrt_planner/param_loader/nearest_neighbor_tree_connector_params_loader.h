/**
 * @file nearest_neighbor_tree_connector_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector_params.h"

namespace rrt_planner::param_loader {

inline rrt_planner::planner_core::nearest_neighbor_tree_connector::
    NearestNeighborTreeConnectorParams
    loadNearestNeighborTreeConnectorParams(ros::NodeHandle* nh,
                                           double costmap_resolution) {
  rrt_planner::planner_core::nearest_neighbor_tree_connector::
      NearestNeighborTreeConnectorParams nearest_neighbor_tree_connector_params;
  loadParam<double>(
      nh, "tree_connection_max_length",
      nearest_neighbor_tree_connector_params.tree_connection_max_length);

  // Scaling with costmap resolution
  nearest_neighbor_tree_connector_params.tree_connection_max_length /=
      costmap_resolution;
  return nearest_neighbor_tree_connector_params;
}
}  // namespace rrt_planner::param_loader

#endif  // RRT_PLANNER__PARAM_LOADER__NEAREST_NEIGHBOR_TREE_CONNECTOR_PARAMS_LOADER_H_
