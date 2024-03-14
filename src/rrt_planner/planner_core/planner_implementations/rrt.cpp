/**
 * @file rrt.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic RRT planner implementation
 * @version 0.1
 * @date 2024-02-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rrt_planner/planner_core/planner_implementations/rrt.h"

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
typename RRT<StateT>::PlanningResultT RRT<StateT>::createPath() {
  unsigned int expansion_index;
  unsigned int target_index = this->goal_->getIndex();
  NodePtr new_node{nullptr};

  PlannerT::setPlanningStartTime();

  while (!PlannerT::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = state_sampler_->generateTreeExpansionIndex(target_index);

    // 2. Extend search tree with new index
    new_node = expander_->expandTree(expansion_index, start_tree_.get(),
                                     this->graph_.get());

    // 3. If expansion was successful, check if new node is target
    if (new_node != nullptr && isGoal(new_node)) {
      PlannerT::logSuccessfulPathCreation();
      return std::make_optional<StateVector>(preparePath(new_node));
    }
  }

  PlannerT::logUnsuccessfulPathCreation();
  return std::nullopt;
}

template <typename StateT>
typename RRT<StateT>::StateVector RRT<StateT>::preparePath(NodePtr node) {
  StateVector raw_path = node->backTracePath();
  std::reverse(raw_path.begin(), raw_path.end());

  if (raw_path.size() < 2) {
    return raw_path;
  }

  StateVector interpolated_path, path_segment;

  for (auto first_it = raw_path.begin(),
            second_it = std::next(raw_path.begin());
       second_it != raw_path.end(); first_it++, second_it++) {
    path_segment = state_connector_->connectStates(*first_it, *second_it);
    std::move(path_segment.begin(), path_segment.end(),
              std::back_inserter(interpolated_path));
  }

  return interpolated_path;
}

template <typename StateT>
typename RRT<StateT>::TreeT RRT<StateT>::transformSearchTree(
    const SearchTreePtr& tree) const {
  TreeT transformed_tree;
  std::vector<std::pair<unsigned int, unsigned int>> edges =
      tree->getTreeEdges();
  transformed_tree.reserve(edges.size());

  std::transform(edges.cbegin(), edges.cend(),
                 std::back_inserter(transformed_tree),
                 [this](const std::pair<unsigned int, unsigned int>& edge) {
                   return state_connector_->connectStates(
                       state_space_->getState(edge.first),
                       state_space_->getState(edge.second));
                 });

  return transformed_tree;
}
}  // namespace rrt_planner::planner_core::planner_implementations

template class rrt_planner::planner_core::planner_implementations::RRT<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::planner_implementations::RRT<
    state_space::state_space_hybrid::StateHybrid>;
