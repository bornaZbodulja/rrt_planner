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
std::optional<std::vector<StateT>> BidirectionalRRT<StateT>::createPath() {
  unsigned int expansion_index;
  bool expanding_start_tree{true};
  NodeT* new_node{nullptr};
  NodeT* closest_node{nullptr};

  RRTCore<StateT>::setPlanningStartTime();

  while (!RRTCore<StateT>::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index =
        expanding_start_tree
            ? state_sampler_->generateTreeExpansionIndex(goal_index_)
            : state_sampler_->generateTreeExpansionIndex(start_index_);

    // 2. Extend search tree with new index
    new_node = expanding_start_tree
                   ? expander_->expandTree(expansion_index, start_tree_.get(),
                                           graph_.get())
                   : expander_->expandTree(expansion_index, goal_tree_.get(),
                                           graph_.get());

    // 3. If expansion was successful, check if new node can be connected to the
    // other tree
    if (new_node != nullptr) {
      closest_node =
          expanding_start_tree
              ? tree_connector_->tryConnectTrees(new_node, goal_tree_.get())
              : tree_connector_->tryConnectTrees(new_node, start_tree_.get());

      if (closest_node != nullptr) {
        std::vector<StateT> path = preparePath(new_node, closest_node);

        if (!expanding_start_tree) {
          std::reverse(path.begin(), path.end());
        }

        RRTCore<StateT>::logSuccessfulPathCreation();
        return std::make_optional<std::vector<StateT>>(path);
      }
    }

    // 4. Switch to other tree
    expanding_start_tree = !expanding_start_tree;
  }

  RRTCore<StateT>::logUnsuccessfulPathCreation();
  return std::nullopt;
}

template <typename StateT>
std::vector<StateT> BidirectionalRRT<StateT>::preparePath(NodeT* node_a,
                                                          NodeT* node_b) {
  std::vector<StateT> current_segment, full_path;

  // Backtracking to root for first tree
  current_segment =
      rrt_planner::planner_core::planner_utilities::backtrackPathFromNodeToRoot(
          node_a, state_connector_.get());
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(full_path));

  // Connection between trees
  current_segment =
      state_connector_->connectStates(node_a->state, node_b->state);
  std::move(current_segment.begin(), current_segment.end(),
            std::back_inserter(full_path));

  // Backtracking to root for second tree
  current_segment =
      rrt_planner::planner_core::planner_utilities::backtrackPathFromNodeToRoot(
          node_b, state_connector_.get());
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
