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
#include <functional>
#include <memory>

#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "rrt_planner/planner_core/planner_implementations/rrt.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
class BidirectionalRRT : public RRT<StateT> {
 public:
  using PlannerT = rrt_planner::planner_core::planner::Planner<StateT>;
  using RRTBaseT = RRT<StateT>;
  using NodeT = typename RRTBaseT::NodeT;
  using NodePtr = typename RRTBaseT::NodePtr;
  using StateVector = typename RRTBaseT::StateVector;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = std::shared_ptr<StateConnectorT>;
  using StateSamplerPtr = typename RRTBaseT::StateSamplerPtr;
  using ExpanderPtr = typename RRTBaseT::ExpanderPtr;
  using TreeConnectorT =
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>;
  using TreeConnectorPtr = std::unique_ptr<TreeConnectorT>;
  using SearchTreeT =
      rrt_planner::planner_core::planner_entities::SearchTree<NodeT>;
  using SearchTreePtr = std::unique_ptr<SearchTreeT>;
  using DistanceGetterT = std::function<double(unsigned int, unsigned int)>;
  using PlanningResultT = typename RRTBaseT::PlanningResultT;
  using SearchPolicyT = typename RRTBaseT::SearchPolicyT;
  using SearchParamsT = typename RRTBaseT::SearchParamsT;
  using TreeVectorT = typename RRTBaseT::TreeVectorT;

  BidirectionalRRT(SearchPolicyT search_policy, SearchParamsT&& search_params,
                   const StateSpacePtr& state_space,
                   const StateConnectorPtr& state_connector,
                   ExpanderPtr&& expander, TreeConnectorPtr&& tree_connector,
                   StateSamplerPtr&& state_sampler)
      : RRTBaseT(search_policy, std::move(search_params), state_space,
                 state_connector, std::move(expander),
                 std::move(state_sampler)),
        tree_connector_(std::move(tree_connector)) {
    // Initialize goal search tree
    RRTBaseT::initializeSearchTree(goal_tree_);
  }

  ~BidirectionalRRT() override = default;

  void initializeSearch() override {
    RRTBaseT::initializeSearch();
    goal_tree_->clear();
  }

  void setStart(const StateT& start_state) override {
    RRTBaseT::setStart(start_state);
    goal_tree_->setTargetNode(this->start_);
  }

  void setGoal(const StateT& goal_state) override {
    RRTBaseT::setGoal(goal_state);
    goal_tree_->setRootNode(this->goal_);
  }

  PlanningResultT createPath() override;

  TreeVectorT getTrees() const override {
    return {RRTBaseT::transformSearchTree(this->start_tree_),
            RRTBaseT::transformSearchTree(goal_tree_)};
  }

 protected:
  /**
   * @brief Backtraces path from given tree nodes to root tree nodes and creates
   * creates connection between trees
   * @param node_a Node from first search tree
   * @param node_b Nearest valid neighbor from second search tree
   * @return StateVector
   */
  StateVector preparePath(NodePtr node_a, NodePtr node_b);

  // Tree connector pointer
  TreeConnectorPtr tree_connector_;
  // Goal search tree pointer
  SearchTreePtr goal_tree_;
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__BIDIRECTIONAL_RRT_H_
