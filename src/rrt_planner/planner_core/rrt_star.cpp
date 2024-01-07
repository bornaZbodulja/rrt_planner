/**
 * @file rrt_star.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT* algorithm implementation
 * @version 0.1
 * @date 2024-01-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rrt_planner/planner_core/rrt_star.h"

#include <ros/console.h>

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core {

template <typename StateT>
typename RRTStar<StateT>::PlanningResultT RRTStar<StateT>::createPath() {
  // Preallocating variables
  unsigned int expansion_index;
  unsigned int target_index = RRTCoreT::getGoalIndex();
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodeVector near_nodes{};
  NodePtr parent_node{nullptr};
  bool tree_expansion_result{false};

  RRTCoreT::setPlanningStartTime();

  while (!RRTCoreT::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = RRTCoreT::getNewExpansionIndex(target_index);

    // 2. Expand search tree with new index
    tree_expansion_result = expandTree(expansion_index, start_tree_, new_node,
                                       closest_node, near_nodes, parent_node);

    // 3. If expansion was successful, check if new node is target
    if (tree_expansion_result && RRTCoreT::isGoal(new_node)) {
      // TODO: Extract this log to method in RRTCore so all planners can use it
      ROS_INFO(
          "RRT* planner found path, used iterations %d/%d, planning time: %.3f "
          "seconds.",
          RRTCoreT::getCurrentExpansionIterationCounter(),
          RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

      return std::make_optional<StateVector>(
          RRTBaseT::backTracePathToRoot(new_node));
    }
  }

  ROS_WARN(
      "RRT* planner unable to find path, used iterations: %d/%d, planning "
      "time: "
      "%.3f seconds.",
      RRTCoreT::getCurrentExpansionIterationCounter(),
      RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

  return {};
}

template <typename StateT>
bool RRTStar<StateT>::expandTree(unsigned int index, SearchTreePtr& tree,
                                 NodePtr& new_node, NodePtr& closest_node,
                                 NodeVector& near_nodes, NodePtr& parent_node) {
  new_node = nullptr;
  parent_node = nullptr;
  near_nodes.clear();
  // Get closest node(state) to indexed state in state space
  closest_node = RRTCoreT::getClosestNode(index, tree);

  if (closest_node == nullptr) {
    return false;
  }

  // Try extending indexed state towards closest node(state)
  const auto extension_result = state_connector_->expandState(
      closest_node->state, state_space_->getState(index));

  // Extending failed, returning
  if (!extension_result.has_value()) {
    new_node = nullptr;
    closest_node = nullptr;
    return false;
  }

  const auto new_node_state = extension_result.value();
  const auto new_node_index = state_space_->getIndex(new_node_state);

  // Get node which corresponds to extension state
  new_node = RRTCoreT::getNodeFromGraph(new_node_index);

  // If node is visited and not in tree return
  if (new_node->isVisited() && !tree->isNodeInTree(new_node)) {
    new_node = nullptr;
    closest_node = nullptr;
    return false;
  }

  new_node->setCellCost(
      collision_checker_->getCost(static_cast<unsigned int>(new_node_state.x),
                                  static_cast<unsigned int>(new_node_state.y)));
  new_node->state = new_node_state;

  // Get near nodes for new node and select parent node for new node among them
  RRTCoreT::getNearNodes(new_node_index, near_nodes, tree);
  parent_node = selectBestParent(new_node, near_nodes);

  // If parent node wasn't found among near nodes, use closest node as parent
  // node
  if (parent_node == nullptr) {
    parent_node = closest_node;
  }

  // Backtrack through parent node ancestors
  parent_node = RRTCoreT::backTracking(new_node, parent_node);

  // Compute cost of expanding towards new node
  const auto accumulated_cost =
      RRTCoreT::getCostmapCost(new_node) +
      RRTCoreT::getTraversalCost(parent_node, new_node) +
      parent_node->getAccumulatedCost();

  const auto new_node_visited = new_node->isVisited();
  const auto update_node =
      !new_node_visited || (accumulated_cost < new_node->getAccumulatedCost());

  if (update_node) {
    new_node->parent = parent_node;
    new_node->setAccumulatedCost(accumulated_cost);
    if (!new_node_visited) {
      new_node->visited();
      RRTCoreT::addNodeToTree(new_node, tree);
    }

    // If enabled, rewire tree around new node
    if (search_info_.rewire_tree) {
      rewireNodes(new_node, near_nodes);
    }

    return true;
  }

  new_node = nullptr;
  closest_node = nullptr;
  near_nodes.clear();
  parent_node = nullptr;
  return false;
}

template <typename StateT>
typename RRTStar<StateT>::NodePtr RRTStar<StateT>::selectBestParent(
    NodePtr& node, NodeVector& potential_parents) {
  if (node == nullptr && potential_parents.empty()) {
    return nullptr;
  }

  NodePtr best_parent{nullptr};
  double min_cost{std::numeric_limits<double>::max()};
  double current_cost{0.0};

  std::for_each(potential_parents.begin(), potential_parents.end(),
                [&](auto& potential_parent) {
                  current_cost =
                      potential_parent->getAccumulatedCost() +
                      RRTCoreT::getTraversalCost(potential_parent, node);

                  if (current_cost < min_cost &&
                      state_connector_->tryConnectStates(
                          potential_parent->state, node->state)) {
                    min_cost = current_cost;
                    best_parent = potential_parent;
                  }
                });

  return best_parent;
}

template <typename StateT>
void RRTStar<StateT>::rewireNodes(NodePtr& potential_parent,
                                  NodeVector& potential_children) {
  if (potential_parent == nullptr || potential_children.empty()) {
    return;
  }

  double new_approach_cost{0.0};

  std::for_each(
      potential_children.begin(), potential_children.end(),
      [&](auto& potential_child) {
        new_approach_cost =
            potential_parent->getAccumulatedCost() +
            RRTCoreT::getTraversalCost(potential_parent, potential_child);

        if (new_approach_cost < potential_child->getAccumulatedCost() &&
            state_connector_->tryConnectStates(potential_parent->state,
                                               potential_child->state)) {
          potential_child->rewireNode(potential_parent, new_approach_cost);
        }
      });
}

}  // namespace rrt_planner::planner_core

// Instantiate algorithm for the supported template types
template class rrt_planner::planner_core::RRTStar<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::RRTStar<
    state_space::state_space_hybrid::StateHybrid>;
