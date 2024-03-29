/**
 * @file bidirectional_rrt.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT planner implementation
 * @version 0.1
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__BIDIRECTIONAL_RRT_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__BIDIRECTIONAL_RRT_H_

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
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
class BidirectionalRRT : public RRTCore<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;
  using SearchTreeT =
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>;

  BidirectionalRRT(
      rrt_planner::planner_core::planner_implementations::SearchPolicy
          search_policy,
      rrt_planner::planner_core::planner_implementations::SearchParams&&
          search_params,
      const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
      const std::shared_ptr<
          state_space::state_connector::StateConnector<StateT>>&
          state_connector,
      std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>&&
          expander,
      std::unique_ptr<
          rrt_planner::planner_core::tree_connector::TreeConnector<StateT>>&&
          tree_connector,
      std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
          state_sampler)
      : RRTCore<StateT>(search_policy, std::move(search_params)),
        state_space_(state_space),
        state_connector_(state_connector),
        expander_(std::move(expander)),
        tree_connector_(std::move(tree_connector)),
        state_sampler_(std::move(state_sampler)),
        graph_(std::make_unique<rrt_planner::planner_core::planner_entities::
                                    SearchGraph<StateT>>()),
        start_tree_(
            rrt_planner::planner_core::planner_utilities::createSearchTree<
                StateT>(state_space_.get())),
        goal_tree_(
            rrt_planner::planner_core::planner_utilities::createSearchTree<
                StateT>(state_space_.get())) {
    graph_->reserve(this->search_params_.max_expansion_iterations);
    start_tree_->reserve(this->search_params_.max_expansion_iterations);
    goal_tree_->reserve(this->search_params_.max_expansion_iterations);
  }

  ~BidirectionalRRT() override = default;

  void initializeSearch() override {
    RRTCore<StateT>::initializeSearch();
    graph_->clear();
    start_tree_->clear();
    goal_tree_->clear();
  }

  void setStart(const StateT& start_state) override {
    NodeT* start = graph_->getNode(start_state);
    start->setAccumulatedCost(0.0);
    start->visited();
    start_tree_->setRootNode(start);
    goal_tree_->setTargetNode(start);
    start_state_ = start->getState();
  }

  void setGoal(const StateT& goal_state) override {
    NodeT* goal = graph_->getNode(goal_state);
    goal->setAccumulatedCost(0.0);
    goal->visited();
    start_tree_->setTargetNode(goal);
    goal_tree_->setRootNode(goal);
    goal_state_ = goal->getState();
  }

  std::optional<std::vector<StateT>> createPath() override;

  std::vector<std::vector<std::vector<StateT>>> getTrees() const override {
    return {rrt_planner::planner_core::planner_utilities::transformSearchTree(
                start_tree_.get(), state_space_.get(), state_connector_.get()),
            rrt_planner::planner_core::planner_utilities::transformSearchTree(
                goal_tree_.get(), state_space_.get(), state_connector_.get())};
  }

 private:
  /**
   * @brief Backtraces path from given tree nodes to root tree nodes and creates
   * creates connection between trees
   * @param node_a Node from first search tree
   * @param node_b Nearest valid neighbor from second search tree
   * @return StateVector
   */
  std::vector<StateT> preparePath(NodeT* node_a, NodeT* node_b);

  // State space pointer
  std::shared_ptr<state_space::StateSpace<StateT>> state_space_;
  // State connector pointer
  std::shared_ptr<state_space::state_connector::StateConnector<StateT>>
      state_connector_;
  // Expander pointer
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander_;
  // Tree connector pointer
  std::unique_ptr<
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>>
      tree_connector_;
  // State sampler pointer
  std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
      state_sampler_;
  // Search graph pointer
  std::unique_ptr<
      rrt_planner::planner_core::planner_entities::SearchGraph<StateT>>
      graph_;
  // Start search tree pointer
  std::unique_ptr<SearchTreeT> start_tree_;
  // Goal search tree pointer
  std::unique_ptr<SearchTreeT> goal_tree_;
  // Start state
  StateT start_state_;
  // Goal state
  StateT goal_state_;
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__BIDIRECTIONAL_RRT_H_
