/**
 * @file rrt.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic RRT planner implementation
 * @version 0.1
 * @date 2024-02-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_H_

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_graph.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "rrt_planner/planner_core/planner_implementations/rrt_core.h"
#include "rrt_planner/planner_core/planner_implementations/search_params.h"
#include "rrt_planner/planner_core/planner_implementations/search_policy.h"
#include "rrt_planner/planner_core/planner_utilities/entities_utilities.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
class RRT : public RRTCore<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;
  using SearchTreeT =
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>;

  RRT(rrt_planner::planner_core::planner_implementations::SearchPolicy
          search_policy,
      rrt_planner::planner_core::planner_implementations::SearchParams&&
          search_params,
      const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
      const std::shared_ptr<
          state_space::state_connector::StateConnector<StateT>>&
          state_connector,
      std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>&&
          expander,
      std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
          state_sampler)
      : RRTCore<StateT>(search_policy, std::move(search_params)),
        state_space_(state_space),
        state_connector_(state_connector),
        expander_(std::move(expander)),
        state_sampler_(std::move(state_sampler)),
        graph_(std::make_unique<rrt_planner::planner_core::planner_entities::
                                    SearchGraph<StateT>>()),
        start_tree_(
            rrt_planner::planner_core::planner_utilities::createSearchTree<
                StateT>(state_space_.get())) {
    graph_->reserve(this->search_params_.max_expansion_iterations);
    start_tree_->reserve(this->search_params_.max_expansion_iterations);
  }

  ~RRT() override = default;

  void initializeSearch() override {
    RRTCore<StateT>::initializeSearch();
    graph_->clear();
    start_tree_->clear();
  }

  void setStart(const StateT& start_state) override {
    NodeT* start = graph_->getNode(start_state);
    start->setAccumulatedCost(0.0);
    start->visited();
    start_tree_->addVertex(start);
  }

  void setGoal(const StateT& goal_state) override {
    NodeT* goal = graph_->getNode(goal_state);
    goal->setAccumulatedCost(0.0);
    goal->visited();
    goal_state_ = goal->getState();
  }

  std::optional<std::vector<StateT>> createPath() override;

  std::vector<std::vector<std::vector<StateT>>> getTrees() const override {
    return {rrt_planner::planner_core::planner_utilities::transformSearchTree(
        start_tree_.get(), state_space_.get(), state_connector_.get())};
  }

 private:
  /**
   * @brief Checks whether given node has same state as goal node
   * @param node Given node
   * @return True if node has same state as goal node, false otherwise
   */
  bool isGoal(const NodeT* node) const {
    return node->getState() == goal_state_;
  }

  // State space pointer
  std::shared_ptr<state_space::StateSpace<StateT>> state_space_;
  // State connector pointer
  std::shared_ptr<state_space::state_connector::StateConnector<StateT>>
      state_connector_;
  // Expander pointer
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander_;
  // State sampler pointer
  std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
      state_sampler_;
  // Search graph pointer
  std::unique_ptr<
      rrt_planner::planner_core::planner_entities::SearchGraph<StateT>>
      graph_;
  // Start search tree pointer
  std::unique_ptr<SearchTreeT> start_tree_;
  // Goal state
  StateT goal_state_;
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_H_
