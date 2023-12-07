/**
 * @file ros_planner_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__ROS_FACTORY__ROS_PLANNER_FACTORY_H_
#define RRT_PLANNER__ROS_FACTORY__ROS_PLANNER_FACTORY_H_

#include <nav_utils/nav_utils.h>

#include <memory>

#include "rrt_planner/param_loader/search_params_loader.h"
#include "rrt_planner/planner_core/rrt_core.h"
#include "rrt_planner/planner_factory/planner_factory.h"
#include "rrt_planner/ros_factory/ros_state_sampler_factory.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/sampling_policy.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::ros_factory {
class ROSPlannerFactory {
 public:
  using SearchPolicyT = rrt_planner::planner_core::SearchPolicy;
  using SamplingPolicyT = state_space::state_sampler::SamplingPolicy;
  template <typename StateT>
  using StateSpaceT = state_space::StateSpace<StateT>;
  template <typename StateT>
  using StateSpacePtr = std::shared_ptr<StateSpaceT<StateT>>;
  template <typename StateT>
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  template <typename StateT>
  using StateConnectorPtr = std::unique_ptr<StateConnectorT<StateT>>;
  template <typename StateT>
  using RRTPlannerT = rrt_planner::planner_core::RRTCore<StateT>;
  template <typename StateT>
  using RRTPlannerPtr = std::unique_ptr<RRTPlannerT<StateT>>;
  using PlannerFactoryT = rrt_planner::planner_factory::PlannerFactory;

  ROSPlannerFactory() = delete;

  /**
   * @brief
   * @tparam StateT
   * @param search_policy
   * @param sampling_policy
   * @param state_space
   * @param state_connector
   * @param collision_checker
   * @param nh
   * @return RRTPlannerPtr<StateT>
   */
  template <typename StateT>
  static RRTPlannerPtr<StateT> createPlanner(
      SearchPolicyT search_policy, SamplingPolicyT sampling_policy,
      const StateSpacePtr<StateT>& state_space,
      StateConnectorPtr<StateT>&& state_connector,
      const CollisionCheckerPtr& collision_checker, double costmap_resolution,
      ros::NodeHandle* nh) {
    auto&& state_sampler = ROSStateSamplerFactory::createSampler<StateT>(
        sampling_policy, state_space->getStateSpaceSize(), nh);
    auto&& search_params = rrt_planner::param_loader::loadSearchParams(
        nh, search_policy, costmap_resolution);
    return PlannerFactoryT::createPlanner<StateT>(
        search_policy, state_space, std::move(state_connector),
        std::move(state_sampler), std::move(search_params), collision_checker);
  }
};
}  // namespace rrt_planner::ros_factory

#endif
