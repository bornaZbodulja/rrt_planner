/**
 * @file nearest_neighbor_tree_connector.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Nearest neighbor tree connector implementation
 * @version 0.1
 * @date 2024-02-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_H_
#define RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_H_

#include <memory>

#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector_params.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"

namespace rrt_planner::planner_core::nearest_neighbor_tree_connector {
template <typename StateT>
class NearestNeighborTreeConnector
    : public rrt_planner::planner_core::tree_connector::TreeConnector<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

  NearestNeighborTreeConnector(
      NearestNeighborTreeConnectorParams&& tree_connector_params,
      const std::shared_ptr<
          state_space::state_connector::StateConnector<StateT>>&
          state_connector)
      : rrt_planner::planner_core::tree_connector::TreeConnector<StateT>(),
        tree_connector_params_(std::move(tree_connector_params)),
        state_connector_(state_connector) {}

  ~NearestNeighborTreeConnector() override = default;

  NodeT* tryConnectTrees(const NodeT* node,
                         const rrt_planner::planner_core::planner_entities::
                             SearchTree<StateT>* const tree) override {
    if (node == nullptr) {
      return nullptr;
    }

    // Find closest node in the second tree
    NodeT* closest_node = tree->getClosestNode(node->getState());

    if (closest_node == nullptr) {
      return nullptr;
    }

    // If distance between nodes(states) greater than max connection length,
    // return
    if (this->state_connector_->getStatesDistance(node->getState(),
                                                  closest_node->getState()) >
        tree_connector_params_.tree_connection_max_length) {
      return nullptr;
    }

    // Check if connection is valid
    if (!state_connector_->tryConnectStates(node->getState(),
                                            closest_node->getState())) {
      return nullptr;
    }

    return closest_node;
  }

 private:
  // Nearest neighbor tree connector parameters
  NearestNeighborTreeConnectorParams tree_connector_params_;
  // State connector pointer
  std::shared_ptr<state_space::state_connector::StateConnector<StateT>>
      state_connector_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_tree_connector

#endif  // RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_H_
