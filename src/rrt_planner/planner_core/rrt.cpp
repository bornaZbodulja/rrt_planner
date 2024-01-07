/**
 * @file rrt.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic RRT algorithm implementation
 * @version 0.1
 * @date 2023-10-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/planner_core/rrt.h"

#include <ros/console.h>

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core {

template <typename StateT>
typename RRT<StateT>::PlanningResultT RRT<StateT>::createPath() {
  // Preallocating variables
  unsigned int expansion_index;
  unsigned int target_index = RRTCoreT::getGoalIndex();
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodePtr parent_node{nullptr};
  bool tree_expansion_result{false};

  RRTCoreT::setPlanningStartTime();

  while (!RRTCoreT::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = RRTCoreT::getNewExpansionIndex(target_index);

    // 2. Extend search tree with new index
    tree_expansion_result = expandTree(expansion_index, start_tree_, new_node,
                                       closest_node, parent_node);

    // 3. If expansion was successful, check if new node is target
    if (tree_expansion_result && RRTCoreT::isGoal(new_node)) {
      // TODO: Extract this log to method in RRTCore so all planners can use it
      ROS_INFO(
          "RRT planner found path, used iterations %d/%d, "
          "planning time: %.3f seconds.",
          RRTCoreT::getCurrentExpansionIterationCounter(),
          RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

      return std::make_optional<StateVector>(backTracePathToRoot(new_node));
    }
  }

  ROS_WARN(
      "RRT planner unable to find path, used iterations: %d/%d, planning time: "
      "%.3f seconds.",
      RRTCoreT::getCurrentExpansionIterationCounter(),
      RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

  return {};
}

template <typename StateT>
bool RRT<StateT>::expandTree(unsigned int index, SearchTreePtr& tree,
                             NodePtr& new_node, NodePtr& closest_node,
                             NodePtr& parent_node) {
  new_node = nullptr;
  parent_node = nullptr;
  // Get closest node(state) to indexed state in state space
  closest_node = RRTCoreT::getClosestNode(index, tree);

  if (closest_node == nullptr) {
    return false;
  }

  // Try extending indexed state towards closest node(state)
  const auto extension_res = state_connector_->expandState(
      closest_node->state, state_space_->getState(index));

  // Extending failed, returning
  if (!extension_res.has_value()) {
    new_node = nullptr;
    closest_node = nullptr;
    return false;
  }

  const auto new_node_state = extension_res.value();

  // Get node which corresponds to extension state
  new_node = RRTCoreT::getNodeFromGraph(state_space_->getIndex(new_node_state));

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

  // Backtrack through closest node ancestors
  parent_node = RRTCoreT::backTracking(new_node, closest_node);

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
    return true;
  }

  new_node = nullptr;
  closest_node = nullptr;
  parent_node = nullptr;
  return false;
}

template <typename StateT>
typename RRT<StateT>::StateVector RRT<StateT>::backTracePathToRoot(
    NodePtr& node) {
  auto state_vector = node->backTracePath();
  std::reverse(state_vector.begin(), state_vector.end());

  if (state_vector.size() < 2) {
    return state_vector;
  }

  StateVector interpolated_path, path_segment;
  auto first_it = state_vector.begin();
  auto second_it = std::next(state_vector.begin());

  while (second_it != state_vector.end()) {
    path_segment = state_connector_->connectStates(*first_it, *second_it);
    std::move(path_segment.begin(), path_segment.end(),
              std::back_inserter(interpolated_path));
    first_it++;
    second_it++;
  }

  return interpolated_path;
}

}  // namespace rrt_planner::planner_core

// Instantiate algorithm for the supported template types
template class rrt_planner::planner_core::RRT<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::RRT<
    state_space::state_space_hybrid::StateHybrid>;
