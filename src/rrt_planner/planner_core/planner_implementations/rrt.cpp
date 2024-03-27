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

#include <functional>

#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
std::optional<std::vector<StateT>> RRT<StateT>::createPath() {
  unsigned int expansion_index;
  NodeT* new_node{nullptr};

  RRTCore<StateT>::setPlanningStartTime();

  while (!RRTCore<StateT>::planningExpired()) {
    // 1. Get new index for tree expansion
    expansion_index = state_sampler_->generateTreeExpansionIndex(goal_index_);

    // 2. Extend search tree with new index
    new_node =
        expander_->expandTree(expansion_index, start_tree_.get(), graph_.get());

    // 3. If expansion was successful, check if new node is target
    if (new_node != nullptr && isGoal(new_node)) {
      RRTCore<StateT>::logSuccessfulPathCreation();
      return std::make_optional<std::vector<StateT>>(
          rrt_planner::planner_core::planner_utilities::
              backtrackPathFromNodeToRoot(new_node, state_connector_.get()));
    }
  }

  RRTCore<StateT>::logUnsuccessfulPathCreation();
  return std::nullopt;
}
}  // namespace rrt_planner::planner_core::planner_implementations

template class rrt_planner::planner_core::planner_implementations::RRT<
    state_space::state_space_2d::State2D>;
template class rrt_planner::planner_core::planner_implementations::RRT<
    state_space::state_space_hybrid::StateHybrid>;
