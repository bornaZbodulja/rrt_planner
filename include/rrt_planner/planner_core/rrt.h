/**
 * @file rrt.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic RRT algorithm implementation
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__RRT_H_
#define RRT_PLANNER__PLANNER_CORE__RRT_H_

#include "rrt_planner/planner_core/rrt_core.h"
#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_core {
template <typename StateT>
class RRT : public RRTCore<StateT> {
 public:
  using RRTBase = RRTCore<StateT>;
  using NodePtr = typename RRTBase::NodePtr;
  using StateVector = typename RRTBase::StateVector;
  using StateSpacePtr = typename RRTBase::StateSpacePtr;
  using StateConnectorPtr = typename RRTBase::StateConnectorPtr;
  using StateSamplerPtr = typename RRTBase::StateSamplerPtr;
  using SearchTreePtr = typename RRTBase::SearchTreePtr;
  using SearchInfo = typename RRTBase::SearchInfo;
  using PlanningResultT = typename RRTBase::PlanningResultT;
  using RRTBase::collision_checker_;
  using RRTBase::start_tree_;
  using RRTBase::state_connector_;
  using RRTBase::state_space_;

  /**
   * @brief
   * @param state_space
   * @param state_connector
   * @param search_info
   * @param collision_checker
   */
  RRT(const StateSpacePtr& state_space, StateConnectorPtr&& state_connector,
      StateSamplerPtr&& state_sampler, SearchInfo&& search_info,
      const CollisionCheckerPtr& collision_checker)
      : RRTBase(state_space, std::move(state_connector),
                std::move(state_sampler), std::move(search_info),
                collision_checker) {}

  PlanningResultT createPath() override;

 protected:
  /**
   * @brief Tries expanding search tree with new node which is computed by
   * extending sampled state in state space towards closest node
   * @param index Index of sampled state in state space
   * @param tree Search tree
   * @param new_node Pointer to newly added node to search tree (filled by
   * method)
   * @param closest_node Pointer to closest node of new node (filled by method)
   * @param parent_node Pointer to parent node of new node (filled by method)
   * @return True if search tree was successfully expanded false otherwise
   */
  virtual bool expandTree(const unsigned int& index, SearchTreePtr& tree,
                          NodePtr& new_node, NodePtr& closest_node,
                          NodePtr& parent_node);

  /**
   * @brief Backtracks path from given node to root node
   * @param node
   * @return StateVector
   */
  inline virtual StateVector backTracePath(NodePtr& node) {
    return node->backTracePath();
  };
};
}  // namespace rrt_planner::planner_core

#endif
