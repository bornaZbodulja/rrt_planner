/**
 * @file rrt.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Basic RRT algorithm declaration
 * @version 0.1
 * @date 2023-09-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__RRT_H_
#define RRT_PLANNER__PLANNER_CORE__RRT_H_

#include <nav_utils/nav_utils.h>

#include "rrt_planner/planner_core/rrt_core.h"

namespace rrt_planner::planner_core {
template <typename StateT>
class RRT : public RRTCore<StateT> {
 public:
  using RRTCoreT = RRTCore<StateT>;
  using NodePtr = typename RRTCoreT::NodePtr;
  using StateVector = typename RRTCoreT::StateVector;
  using StateSpacePtr = typename RRTCoreT::StateSpacePtr;
  using StateConnectorPtr = typename RRTCoreT::StateConnectorPtr;
  using StateSamplerPtr = typename RRTCoreT::StateSamplerPtr;
  using SearchTreePtr = typename RRTCoreT::SearchTreePtr;
  using SearchInfo = typename RRTCoreT::SearchInfo;
  using PlanningResultT = typename RRTCoreT::PlanningResultT;
  using RRTCoreT::collision_checker_;
  using RRTCoreT::start_tree_;
  using RRTCoreT::state_connector_;
  using RRTCoreT::state_space_;

  /**
   * @brief Planner constructor
   * @param state_space State space pointer
   * @param state_connector State connector pointer
   * @param state_sampler State sampler pointer
   * @param search_info Search params
   * @param collision_checker Collision checker pointer
   */
  RRT(StateSpacePtr state_space, StateConnectorPtr state_connector,
      StateSamplerPtr state_sampler, SearchInfo search_info,
      CollisionCheckerPtr collision_checker)
      : RRTCoreT(state_space, state_connector, state_sampler,
                 search_info, collision_checker) {}

  ~RRT() override = default;

  PlanningResultT createPath() override;

 protected:
  /**
   * @brief Tries expanding search tree with new node which is computed by
   * extending sampled state in state space towards closest node
   * @param index Index of sampled state in state space
   * @param tree Search tree pointer
   * @param new_node Pointer to newly added node to search tree (filled by
   * method)
   * @param closest_node Pointer to closest node of new node (filled by method)
   * @param parent_node Pointer to parent node of new node (filled by method)
   * @return True if search tree was successfully expanded false otherwise
   */
  virtual bool expandTree(unsigned int index, SearchTreePtr& tree,
                          NodePtr& new_node, NodePtr& closest_node,
                          NodePtr& parent_node);

  /**
   * @brief Backtracks path from given node to root node
   * @param node
   * @return StateVector
   */
  StateVector backTracePathToRoot(NodePtr& node);
};
}  // namespace rrt_planner::planner_core

#endif
