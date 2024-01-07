/**
 * @file rrt_star.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT* algorithm declaration
 * @version 0.1
 * @date 2024-01-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__RRT_STAR_H_
#define RRT_PLANNER__PLANNER_CORE__RRT_STAR_H_

#include <nav_utils/nav_utils.h>

#include <vector>

#include "rrt_planner/planner_core/rrt.h"
#include "rrt_planner/planner_core/rrt_core.h"

namespace rrt_planner::planner_core {
template <typename StateT>
class RRTStar : public RRT<StateT> {
 public:
  using RRTBaseT = RRT<StateT>;
  using RRTCoreT = RRTCore<StateT>;
  using NodePtr = typename RRTCoreT::NodePtr;
  using NodeVector = typename RRTCoreT::NodeVector;
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
  using RRTCoreT::search_info_;

  /**
   * @brief Planner constructor
   * @param state_space State space pointer
   * @param state_connector State connector pointer
   * @param state_sampler State sampler pointer
   * @param search_info Search params
   * @param collision_checker Collision checker pointer
   */
  RRTStar(const StateSpacePtr& state_space, StateConnectorPtr&& state_connector,
          StateSamplerPtr&& state_sampler, SearchInfo&& search_info,
          const CollisionCheckerPtr& collision_checker)
      : RRTBaseT(state_space, std::move(state_connector),
                 std::move(state_sampler), std::move(search_info),
                 collision_checker) {}

  ~RRTStar() override = default;

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
   * @param near_nodes Vector of node in neighborhood of new node (filled by
   * method)
   * @param parent_node Pointer to parent node of new node (filled by method)
   * @return True if search tree was successfully expanded false otherwise
   */
  virtual bool expandTree(unsigned int index, SearchTreePtr& tree,
                          NodePtr& new_node, NodePtr& closest_node,
                          NodeVector& near_nodes, NodePtr& parent_node);

  /**
   * @brief Chooses parent which yields lowest accumulated cost towards given
   * node
   * @param node Given node
   * @param potential_parents Vector of potential parents
   * @return NodePtr
   */
  virtual NodePtr selectBestParent(NodePtr& node,
                                   NodeVector& potential_parents);

  /**
   * @brief For each potential_children nodes checks whether lower cost approach
   * can be made using potential_parent node as parent, and if it can, rewires
   * node to new parent
   * @param potential_parent New potential parent
   * @param potential_children Vector of nodes to check if lower cost approach
   * can be made
   */
  virtual void rewireNodes(NodePtr& potential_parent,
                           NodeVector& potential_children);
};
}  // namespace rrt_planner::planner_core

#endif
