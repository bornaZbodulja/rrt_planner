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

using namespace rrt_planner::planner_core;

template <typename StateT>
typename RRT<StateT>::PlanningResultT RRT<StateT>::createPath() {
  // Preallocating variables
  unsigned int expansion_index;
  unsigned int target_index = RRTBase::getGoalIndex();
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodePtr parent_node{nullptr};
  bool tree_expansion_result{false};

  RRTBase::setPlanningStartTime();

  while (!RRTBase::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = RRTBase::getNewExpansionIndex(target_index);

    // 2. Extend search tree with new index
    tree_expansion_result = expandTree(expansion_index, start_tree_, new_node,
                                       closest_node, parent_node);

    // 3. If expansion was successful, check if new node is target
    if (tree_expansion_result && RRTBase::isGoal(new_node)) {
      ROS_INFO(
          "RRT planner found path, used iterations %d/%d, "
          "planning time: %.3f seconds.",
          RRTBase::getCurrentExpansionIterationCounter(),
          RRTBase::getMaximumNumberOfIterations(), RRTBase::getElapsedTime());

      return std::make_optional<StateVector>(backTracePath(new_node));
    }
  }

  ROS_WARN(
      "RRT planner unable to find path, used iterations: %d/%d, planning time: "
      "%.3f seconds.",
      RRTBase::getCurrentExpansionIterationCounter(),
      RRTBase::getMaximumNumberOfIterations(), RRTBase::getElapsedTime());

  return {};
}

template <typename StateT>
bool RRT<StateT>::expandTree(const unsigned int& index, SearchTreePtr& tree,
                             NodePtr& new_node, NodePtr& closest_node,
                             NodePtr& parent_node) {
  new_node = nullptr;
  // Get closest node(state) to indexed state in state space
  closest_node = RRTBase::getClosestNode(index, tree);

  if (closest_node == nullptr) {
    return false;
  }

  // Try extending indexed state towards closest node(state)
  const auto extension_res = state_connector_->expandState(
      closest_node->state, state_space_->getState(index));

  // Extending failed, returning
  if (!extension_res.has_value()) {
    return false;
  }

  const auto new_node_state = extension_res.value();

  // Get node which corresponds to extension state
  new_node = RRTBase::getNodeFromGraph(state_space_->getIndex(new_node_state));
  new_node->setCellCost(
      collision_checker_->GetCost(static_cast<unsigned int>(new_node_state.x),
                                  static_cast<unsigned int>(new_node_state.y)));
  new_node->state = new_node_state;

  // Backtrack through closest node ancestors
  parent_node = RRTBase::backTracking(new_node, closest_node);

  // Compute cost of expanding towards new node
  const auto accumulated_cost =
      RRTBase::getCostmapCost(new_node) +
      RRTBase::getTraversalCost(parent_node, new_node) +
      parent_node->getAccumulatedCost();

  const auto new_node_visited = new_node->isVisited();
  const auto update_node =
      !new_node_visited || (accumulated_cost < new_node->getAccumulatedCost());

  if (update_node) {
    new_node->parent = parent_node;
    new_node->setAccumulatedCost(accumulated_cost);
    if (!new_node_visited) {
      new_node->visited();
      RRTBase::addNodeToTree(new_node, tree);
    }
    return true;
  }

  return false;
}

// Instantiate algorithm for the supported template types
template class RRT<state_space::state_space_2d::State2D>;
template class RRT<state_space::state_space_hybrid::StateHybrid>;
