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
#include <functional>
#include <memory>

#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_implementations {
template <typename StateT>
class RRT : public rrt_planner::planner_core::planner::Planner<StateT> {
 public:
  using PlannerT = rrt_planner::planner_core::planner::Planner<StateT>;
  using NodeT = typename PlannerT::NodeT;
  using NodePtr = typename PlannerT::NodePtr;
  using StateVector = typename PlannerT::StateVector;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = std::shared_ptr<StateConnectorT>;
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  using StateSamplerPtr = std::unique_ptr<StateSamplerT>;
  using ExpanderT = rrt_planner::planner_core::expander::Expander<StateT>;
  using ExpanderPtr = std::unique_ptr<ExpanderT>;
  using TreeConnectorT =
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>;
  using TreeConnectorPtr = std::unique_ptr<TreeConnectorT>;
  using SearchTreeT =
      rrt_planner::planner_core::planner_entities::SearchTree<NodeT>;
  using SearchTreePtr = std::unique_ptr<SearchTreeT>;
  using DistanceGetterT = std::function<double(unsigned int, unsigned int)>;
  using PlanningResultT = typename PlannerT::PlanningResultT;
  using SearchPolicyT = typename PlannerT::SearchPolicyT;
  using SearchParamsT = rrt_planner::planner_core::planner::SearchParams;
  using TreeVectorT = typename PlannerT::TreeVectorT;
  using TreeT = std::vector<StateVector>;

  RRT(SearchPolicyT search_policy, SearchParamsT&& search_params,
      const StateSpacePtr& state_space,
      const StateConnectorPtr& state_connector, ExpanderPtr&& expander,
      StateSamplerPtr&& state_sampler)
      : PlannerT(search_policy, std::move(search_params)),
        state_space_(state_space),
        state_connector_(state_connector),
        expander_(std::move(expander)),
        state_sampler_(std::move(state_sampler)) {
    initializeSearchTree(start_tree_);
  }

  ~RRT() override = default;

  void initializeSearch() override {
    PlannerT::initializeSearch();
    start_tree_->clear();
  }

  void setStart(const StateT& start_state) override {
    this->start_ =
        PlannerT::getNodeFromGraph(state_space_->getIndex(start_state));
    this->start_->state = start_state;
    this->start_->setAccumulatedCost(0.0);
    this->start_->visited();
    start_tree_->setRootNode(this->start_);
  }

  void setGoal(const StateT& goal_state) override {
    this->goal_ =
        PlannerT::getNodeFromGraph(state_space_->getIndex(goal_state));
    this->goal_->state = goal_state;
    this->goal_->setAccumulatedCost(0.0);
    this->goal_->visited();
    start_tree_->setTargetNode(this->goal_);
  }

  PlanningResultT createPath() override;

  TreeVectorT getTrees() const override {
    return {transformSearchTree(start_tree_)};
  }

 protected:
  /**
   * @brief Initializes search tree with distance getter
   * @param tree Search tree pointer
   */
  void initializeSearchTree(SearchTreePtr& tree) {
    DistanceGetterT distance_getter = [this](unsigned int a,
                                             unsigned int b) -> double {
      return state_space_
          ->getStateDistance(state_space_->getState(a),
                             state_space_->getState(b))
          .squaredL2norm();
    };
    tree = std::make_unique<SearchTreeT>(std::move(distance_getter));
    tree->reserve(this->search_params_.max_expansion_iterations);
  }

  /**
   * @brief Checks whether given node has same state index as goal node
   * @param node Given node
   * @return True if node has same state index as goal node, false otherwise
   */
  bool isGoal(const NodePtr& node) const { return node == this->goal_; }

  /**
   * @brief Backtracks path from given node to root node
   * @param node
   * @return StateVector
   */
  StateVector preparePath(NodePtr node);

  /**
   * @brief Transforms search tree to vector of state vectors representing edges
   * in search tree
   * @param tree Search tree pointer
   * @return TreeT
   */
  TreeT transformSearchTree(const SearchTreePtr& tree) const;

  // State space pointer
  StateSpacePtr state_space_;
  // State connector pointer
  StateConnectorPtr state_connector_;
  // Expander pointer
  ExpanderPtr expander_;
  // State sampler pointer
  StateSamplerPtr state_sampler_;
  // Start search tree pointer
  SearchTreePtr start_tree_;
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_H_
