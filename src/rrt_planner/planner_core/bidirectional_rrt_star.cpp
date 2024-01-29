/**
 * @file bidirectional_rrt_star.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT* implementation
 * @version 0.1
 * @date 2024-01-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rrt_planner/planner_core/bidirectional_rrt_star.h"

#include <ros/console.h>

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core {

template <typename StateT>
typename BidirectionalRRTStar<StateT>::PlanningResultT
BidirectionalRRTStar<StateT>::createPath() {
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
            ? RRTStarT::expandTree(expansion_index, start_tree_, new_node,
                                   closest_node, parent_node)
            : RRTStarT::expandTree(expansion_index, goal_tree_, new_node,
                                   closest_node, parent_node);

    // 3. Try connecting search trees
    tree_connecting_result = expanding_start_tree
                                 ? BidirectionalRRTT::tryConnectTrees(
                                       new_node, closest_node, goal_tree_)
                                 : BidirectionalRRTT::tryConnectTrees(
                                       new_node, closest_node, start_tree_);

    if (tree_expansion_result && tree_connecting_result) {
      // TODO: Extract this log to method in RRTCore so all planners can use it
      ROS_INFO(
          "Bidirectional RRT* planner found path, used iterations %d/%d, "
          "planning time: %.3f seconds.",
          RRTCoreT::getCurrentExpansionIterationCounter(),
          RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

      auto path = BidirectionalRRTT::preparePath(new_node, closest_node);

      if (!expanding_start_tree) {
        std::reverse(path.begin(), path.end());
      }

      return std::make_optional<StateVector>(path);
    }

    // 4. Switch to other tree
    expanding_start_tree = !expanding_start_tree;
  }

  // TODO: Extract this log to method in RRTCore so all planners can use it
  ROS_WARN(
      "Bidirectional RRT* planner unable to find path, used iterations: %d/%d, "
      "planning time: "
      "%.3f seconds.",
      RRTCoreT::getCurrentExpansionIterationCounter(),
      RRTCoreT::getMaximumNumberOfIterations(), RRTCoreT::getElapsedTime());

  return std::nullopt;
}

}  // namespace rrt_planner::planner_core

// Instantiate algorithm for the supported template types
template class rrt_planner::planner_core::BidirectionalRRTStar<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::BidirectionalRRTStar<
    state_space::state_space_hybrid::StateHybrid>;
