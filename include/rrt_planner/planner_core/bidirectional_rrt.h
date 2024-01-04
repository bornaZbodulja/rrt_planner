/**
 * @file bidirectional_rrt.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT algorithm implementation
 * @version 0.1
 * @date 2023-12-10
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__BIDIRECTIONAL_RRT_H_
#define RRT_PLANNER__PLANNER_CORE__BIDIRECTIONAL_RRT_H_

#include "rrt_planner/planner_core/rrt.h"
#include "rrt_planner/planner_core/rrt_core.h"

namespace rrt_planner::planner_core {
template <typename StateT>
class BidirectionalRRT : public RRT<StateT> {
 public:
  using RRTBaseT = RRT<StateT>;
  using RRTCoreT = RRTCore<StateT>;
  using NodePtr = typename RRTBaseT::NodePtr;
  using StateVector = typename RRTBaseT::StateVector;
  using StateSpacePtr = typename RRTBaseT::StateSpacePtr;
  using StateConnectorPtr = typename RRTBaseT::StateConnectorPtr;
  using StateSamplerPtr = typename RRTBaseT::StateSamplerPtr;
  using SearchTreeT = typename RRTCoreT::SearchTreeT;
  using SearchTreePtr = typename RRTBaseT::SearchTreePtr;
  using SearchInfo = typename RRTBaseT::SearchInfo;
  using PlanningResultT = typename RRTBaseT::PlanningResultT;
  using TreeMsgVectorT = typename RRTCoreT::TreeMsgVectorT;
  using RRTCoreT::collision_checker_;
  using RRTCoreT::goal_;
  using RRTCoreT::search_info_;
  using RRTCoreT::start_;
  using RRTCoreT::start_tree_;
  using RRTCoreT::state_connector_;
  using RRTCoreT::state_space_;

  /**
   * @brief
   * @param state_space
   * @param state_connector
   * @param state_sampler
   * @param search_info
   * @param collision_checker
   */
  BidirectionalRRT(const StateSpacePtr& state_space,
                   StateConnectorPtr&& state_connector,
                   StateSamplerPtr&& state_sampler, SearchInfo&& search_info,
                   const CollisionCheckerPtr& collision_checker)
      : RRTBaseT(state_space, std::move(state_connector),
                 std::move(state_sampler), std::move(search_info),
                 collision_checker),
        goal_tree_(std::make_unique<SearchTreeT>()) {}

  ~BidirectionalRRT() override { clearGoalTree(); }

  void initializeSearch() override {
    clearGoalTree();
    RRTCoreT::initializeSearch();
  }

  void setStart(const StateT& start_state) override {
    RRTCoreT::setStart(start_state);
    goal_tree_->setTargetNode(start_);
  }

  void setGoal(const StateT& goal_state) override {
    RRTCoreT::setGoal(goal_state);
    goal_tree_->setRootNode(goal_);
  }

  PlanningResultT createPath() override;

  TreeMsgVectorT getSearchTrees() const override {
    return {RRTCoreT::searchTreeToTreeMsg(start_tree_),
            RRTCoreT::searchTreeToTreeMsg(goal_tree_)};
  }

 protected:
  /**
   * @brief Tries connecting newly added node to first search tree with closest
   * node from second search tree
   * @param new_node Newly added node to search tree
   * @param closest_node Closest node to newly added node in second search tree
   * (filled by method)
   * @param second_tree Second search tree
   * @return Returns true if connecting new node to second search tree was
   * successful, false otherwise
   */
  virtual bool tryConnectTrees(NodePtr& new_node, NodePtr& closest_node,
                               SearchTreePtr& second_tree);

  /**
   * @brief Prepares path by backtracking path from two nodes that connect
   * search trees
   * @param first_node Connection node from first search tree
   * @param second_node Connection node from second search tree
   * @return StateVector
   */
  StateVector preparePath(NodePtr& first_node, NodePtr& second_node);

  void clearGoalTree() { goal_tree_->clear(); }

  SearchTreePtr goal_tree_;
};
}  // namespace rrt_planner::planner_core

#endif
