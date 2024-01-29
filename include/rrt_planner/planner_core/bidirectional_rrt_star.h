/**
 * @file bidirectional_rrt_star.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Bidirectional RRT* algorithm declaration
 * @version 0.1
 * @date 2024-01-07
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__BIDIRECTIONAL_RRT_STAR_H_
#define RRT_PLANNER__PLANNER_CORE__BIDIRECTIONAL_RRT_STAR_H_

#include <nav_utils/nav_utils.h>

#include <memory>

#include "rrt_planner/planner_core/bidirectional_rrt.h"
#include "rrt_planner/planner_core/rrt_core.h"
#include "rrt_planner/planner_core/rrt_star.h"

namespace rrt_planner::planner_core {
template <typename StateT>
class BidirectionalRRTStar : public BidirectionalRRT<StateT>,
                             public RRTStar<StateT> {
 public:
  using RRTCoreT = RRTCore<StateT>;
  using RRTStarT = RRTStar<StateT>;
  using BidirectionalRRTT = BidirectionalRRT<StateT>;
  using NodePtr = typename RRTCoreT::NodePtr;
  using StateVector = typename RRTCoreT::StateVector;
  using StateSpacePtr = typename RRTCoreT::StateSpacePtr;
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = typename RRTCoreT::StateConnectorPtr;
  using StateSamplerPtr = typename RRTCoreT::StateSamplerPtr;
  using SearchInfo = typename RRTCoreT::SearchInfo;
  using PlanningResultT = typename RRTCoreT::PlanningResultT;
  using BidirectionalRRTT::goal_tree_;
  using RRTCoreT::collision_checker_;
  using RRTCoreT::goal_;
  using RRTCoreT::search_info_;
  using RRTCoreT::start_;
  using RRTCoreT::start_tree_;
  using RRTCoreT::state_connector_;
  using RRTCoreT::state_space_;

  BidirectionalRRTStar(StateSpacePtr state_space,
                       StateConnectorPtr state_connector,
                       StateSamplerPtr state_sampler, SearchInfo search_info,
                       CollisionCheckerPtr collision_checker)
      : BidirectionalRRTT(state_space, state_connector, state_sampler,
                          search_info, collision_checker),
        RRTStarT(state_space, state_connector, state_sampler, search_info,
                 collision_checker) {}

  ~BidirectionalRRTStar() override = default;

  PlanningResultT createPath() override;
};
}  // namespace rrt_planner::planner_core

#endif
