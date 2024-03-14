/**
 * @file bidirectional_rrt.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT planner implementation
 * @version 0.1
 * @date 2024-02-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rrt_planner/planner_core/planner_implementations/bidirectional_rrt.h"

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
typename BidirectionalRRT<StateT>::PlanningResultT
BidirectionalRRT<StateT>::createPath() {
  unsigned int expansion_index;
  unsigned int start_index = this->start_->getIndex();
  unsigned int goal_index = this->goal_->getIndex();
  bool expanding_start_tree{true};
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};

  PlannerT::setPlanningStartTime();

  while (!PlannerT::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index =
        expanding_start_tree
            ? this->state_sampler_->generateTreeExpansionIndex(goal_index)
            : this->state_sampler_->generateTreeExpansionIndex(start_index);

    // 2. Extend search tree with new index
    new_node =
        expanding_start_tree
            ? this->expander_->expandTree(
                  expansion_index, this->start_tree_.get(), this->graph_.get())
            : this->expander_->expandTree(expansion_index, goal_tree_.get(),
                                          this->graph_.get());

    // 3. If expansion was successful, check if new node can be connected to the
    // other tree
    if (new_node != nullptr) {
      closest_node =
          expanding_start_tree
              ? tree_connector_->tryConnectTrees(new_node, goal_tree_.get())
              : tree_connector_->tryConnectTrees(new_node,
                                                 this->start_tree_.get());

      if (closest_node != nullptr) {
        StateVector path = preparePath(new_node, closest_node);

        if (!expanding_start_tree) {
          std::reverse(path.begin(), path.end());
        }

        PlannerT::logSuccessfulPathCreation();
        return std::make_optional<StateVector>(path);
      }
    } 

    // 4. Switch to other tree
    expanding_start_tree = !expanding_start_tree;
  }

  PlannerT::logUnsuccessfulPathCreation();
  return std::nullopt;
}

template <typename StateT>
typename BidirectionalRRT<StateT>::StateVector
BidirectionalRRT<StateT>::preparePath(NodePtr node_a, NodePtr node_b) {
  StateVector current_segment, full_path;

  // Backtracking to root for first tree
  current_segment = RRTBaseT::preparePath(node_a);
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(full_path));

  // Connection between trees
  current_segment =
      this->state_connector_->connectStates(node_a->state, node_b->state);
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(full_path));

  // Backtracking to root for second tree
  current_segment = RRTBaseT::preparePath(node_b);
  std::reverse(current_segment.begin(), current_segment.end());
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(full_path));

  return full_path;
}
}  // namespace rrt_planner::planner_core::planner_implementations

template class rrt_planner::planner_core::planner_implementations::
    BidirectionalRRT<state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::planner_implementations::
    BidirectionalRRT<state_space::state_space_hybrid::StateHybrid>;
