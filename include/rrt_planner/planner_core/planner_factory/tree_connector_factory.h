/**
 * @file tree_connector_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Tree connector factory
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__TREE_CONNECTOR_FACTORY_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__TREE_CONNECTOR_FACTORY_H_

#include <memory>

#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector.h"
#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector_params.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"

namespace rrt_planner::planner_core::planner_factory {

template <typename StateT>
std::unique_ptr<
    rrt_planner::planner_core::tree_connector::TreeConnector<StateT>>
createNearestNeighborTreeConnector(
    rrt_planner::planner_core::nearest_neighbor_tree_connector::
        NearestNeighborTreeConnectorParams&& tree_connector_params,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector) {
  return std::make_unique<
      rrt_planner::planner_core::nearest_neighbor_tree_connector::
          NearestNeighborTreeConnector<StateT>>(
      std::move(tree_connector_params), state_connector);
}
}  // namespace rrt_planner::planner_core::planner_factory

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__TREE_CONNECTOR_FACTORY_H_
