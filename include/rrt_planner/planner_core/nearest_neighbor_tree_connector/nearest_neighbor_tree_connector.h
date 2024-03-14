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

#include <optional>
#include <memory>

#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector_params.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"

namespace rrt_planner::planner_core::nearest_neighbor_tree_connector {
template <typename StateT>
class NearestNeighborTreeConnector
    : public rrt_planner::planner_core::tree_connector::TreeConnector<StateT> {
 public:
  using TreeConnectorT =
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>;
  using NodePtr = typename TreeConnectorT::NodePtr;
  using SearchTreePtr = typename TreeConnectorT::SearchTreePtr;
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = std::shared_ptr<StateConnectorT>;

  NearestNeighborTreeConnector(
      NearestNeighborTreeConnectorParams&& tree_connector_params,
      const StateConnectorPtr& state_connector)
      : TreeConnectorT(),
        tree_connector_params_(std::move(tree_connector_params)),
        state_connector_(state_connector) {}

  ~NearestNeighborTreeConnector() override = default;

  NodePtr tryConnectTrees(const NodePtr& node,
                          const SearchTreePtr& tree) override {
    if (node == nullptr) {
      return nullptr;
    }

    // Find closest node in the second tree
    NodePtr closest_node = tree->getClosestNode(node->getIndex());

    if (closest_node == nullptr) {
      return nullptr;
    }

    // If distance between nodes(states) greater than max connection length,
    // return
    if (this->state_connector_->getStatesDistance(node->state,
                                                  closest_node->state) >
        tree_connector_params_.tree_connection_max_length) {
      return nullptr;
    }

    // Check if connection is valid
    if (!state_connector_->tryConnectStates(node->state, closest_node->state)) {
      return nullptr;
    }

    return closest_node;
  }

 protected:
  // Nearest neighbor tree connector parameters
  NearestNeighborTreeConnectorParams tree_connector_params_;
  // State connector pointer
  StateConnectorPtr state_connector_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_tree_connector

#endif  // RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_TREE_CONNECTOR__NEAREST_NEIGHBOR_TREE_CONNECTOR_H_
