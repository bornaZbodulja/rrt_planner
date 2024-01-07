/**
 * @file bidirectional_rrt.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT algorithm implementation
 * @version 0.1
 * @date 2023-12-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/planner_core/bidirectional_rrt.h"

#include <ros/console.h>

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core {

template <typename StateT>
typename BidirectionalRRT<StateT>::PlanningResultT
BidirectionalRRT<StateT>::createPath() {
  // Preallocating variables
  unsigned int expansion_index;
  unsigned int start_index = start_->getIndex();
  unsigned int goal_index = goal_->getIndex();
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodePtr parent_node{nullptr};
  bool expanding_start_tree{true};
  bool tree_expansion_result{false};
  bool tree_connecting_result{false};

  RRTCoreT::setPlanningStartTime();

  while (!RRTCoreT::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = expanding_start_tree
                          ? RRTCoreT::getNewExpansionIndex(goal_index)
                          : RRTCoreT::getNewExpansionIndex(start_index);

    // 2. Extend search tree with new index
    tree_expansion_result =
        expanding_start_tree
            ? RRTBaseT::expandTree(expansion_index, start_tree_, new_node,
                                   closest_node, parent_node)
            : RRTBaseT::expandTree(expansion_index, goal_tree_, new_node,
                                   closest_node, parent_node);

    // 3. Try connecting search trees
    tree_connecting_result =
        expanding_start_tree
            ? tryConnectTrees(new_node, closest_node, goal_tree_)
            : tryConnectTrees(new_node, closest_node, start_tree_);

    if (tree_expansion_result && tree_connecting_result) {
      // TODO: Extract this log to method in RRTCore so all planners can use it
      ROS_INFO(
          "Bidirectional RRT planner found path, used iterations %d/%d, "
          "planning time: %.3f seconds.",
          RRTCoreT::getCurrentExpansionIterationCounter(),
          RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

      auto path = preparePath(new_node, closest_node);

      if (!expanding_start_tree) {
        std::reverse(path.begin(), path.end());
      }

      return std::make_optional<StateVector>(path);
    }

    // 4. Switch to other tree
    expanding_start_tree = !expanding_start_tree;
  }

  ROS_WARN(
      "Bidirectional RRT planner unable to find path, used iterations: %d/%d, "
      "planning time: "
      "%.3f seconds.",
      RRTCoreT::getCurrentExpansionIterationCounter(),
      RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

  return {};
}

template <typename StateT>
bool BidirectionalRRT<StateT>::tryConnectTrees(NodePtr& new_node,
                                               NodePtr& closest_node,
                                               SearchTreePtr& second_tree) {
  if (new_node == nullptr) {
    return false;
  }

  closest_node = RRTCoreT::getClosestNode(new_node->getIndex(), second_tree);

  if (closest_node == nullptr) {
    return false;
  }

  if (state_connector_->getStatesDistance(new_node->state,
                                          closest_node->state) >
      search_info_.tree_connection_max_length) {
    return false;
  }

  return state_connector_->tryConnectStates(new_node->state,
                                            closest_node->state);
}

template <typename StateT>
typename BidirectionalRRT<StateT>::StateVector
BidirectionalRRT<StateT>::preparePath(NodePtr& first_node,
                                      NodePtr& second_node) {
  StateVector current_segment, path;
  current_segment = RRTBaseT::backTracePathToRoot(first_node);
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(path));
  current_segment =
      state_connector_->connectStates(first_node->state, second_node->state);
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(path));
  current_segment = RRTBaseT::backTracePathToRoot(second_node);
  std::reverse(current_segment.begin(), current_segment.end());
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(path));
  return path;
}

}  // namespace rrt_planner::planner_core

// Instantiate algorithm for the supported template types
template class rrt_planner::planner_core::BidirectionalRRT<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::BidirectionalRRT<
    state_space::state_space_hybrid::StateHybrid>;
