/**
 * @file planner_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planner creation factory
 * @version 0.1
 * @date 2023-10-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_FACTORY__PLANNER_FACTORY_H_
#define RRT_PLANNER__PLANNER_FACTORY__PLANNER_FACTORY_H_

#include <nav_utils/nav_utils.h>

#include <memory>

#include "rrt_planner/planner_core/bidirectional_rrt.h"
#include "rrt_planner/planner_core/rrt.h"
#include "rrt_planner/planner_core/rrt_core.h"
#include "rrt_planner/planner_core/search_policy.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"
#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace rrt_planner::planner_factory {
class PlannerFactory {
 public:
  template <typename StateT>
  using StateSpaceT = state_space::StateSpace<StateT>;
  template <typename StateT>
  using StateSpacePtr = std::shared_ptr<StateSpaceT<StateT>>;
  template <typename StateT>
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  template <typename StateT>
  using StateConnectorPtr = std::unique_ptr<StateConnectorT<StateT>>;
  template <typename StateT>
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  template <typename StateT>
  using StateSamplerPtr = std::unique_ptr<StateSamplerT<StateT>>;
  template <typename StateT>
  using RRTPlannerT = rrt_planner::planner_core::RRTCore<StateT>;
  template <typename StateT>
  using RRTPlannerPtr = std::unique_ptr<RRTPlannerT<StateT>>;
  using SearchInfo = rrt_planner::planner_core::SearchParams;
  using SearchPolicyT = rrt_planner::planner_core::SearchPolicy;

  PlannerFactory() = delete;

  /**
   * @brief
   * @tparam StateT
   * @param search_policy
   * @param state_space
   * @param state_connector
   * @param state_sampler
   * @param search_info
   * @param collision_checker
   * @return RRTPlannerPtr<StateT>
   */
  template <typename StateT>
  static RRTPlannerPtr<StateT> createPlanner(
      SearchPolicyT search_policy, const StateSpacePtr<StateT>& state_space,
      StateConnectorPtr<StateT>&& state_connector,
      StateSamplerPtr<StateT>&& state_sampler, SearchInfo&& search_info,
      const CollisionCheckerPtr& collision_checker) {
    switch (search_policy) {
      case SearchPolicyT::RRT:
        return std::make_unique<rrt_planner::planner_core::RRT<StateT>>(
            state_space, std::move(state_connector), std::move(state_sampler),
            std::move(search_info), collision_checker);

      case SearchPolicyT::BIDIRECTIONAL_RRT:
        return std::make_unique<
            rrt_planner::planner_core::BidirectionalRRT<StateT>>(
            state_space, std::move(state_connector), std::move(state_sampler),
            std::move(search_info), collision_checker);

      default:
        return nullptr;
    }
  }
};
}  // namespace rrt_planner::planner_factory

#endif
