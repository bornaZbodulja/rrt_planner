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

#include <nav_utils/collision_checker.h>
#include <ros/node_handle.h>

#include <memory>

#include "rrt_planner/param_loader/cost_scorer_params_loader.h"
#include "rrt_planner/param_loader/nearest_neighbor_star_expander_params_loader.h"
#include "rrt_planner/param_loader/nearest_neighbor_tree_connector_params_loader.h"
#include "rrt_planner/param_loader/search_params_loader.h"
#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/planner_core/planner_factory/planner_factory.h"
#include "rrt_planner/ros_factory/ros_state_sampler_factory.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/sampling_policy.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::ros_factory {

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner::Planner<StateT>>
createPlanner(
    ros::NodeHandle* nh,
    rrt_planner::planner_core::planner::SearchPolicy search_policy,
    state_space::state_sampler::SamplingPolicy sampling_policy,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  rrt_planner::planner_core::planner::SearchParams search_params =
      rrt_planner::param_loader::loadSearchParams(nh);
  rrt_planner::planner_core::cost_scorer::CostScorerParams cost_scorer_params =
      rrt_planner::param_loader::loadCostScorerParams(nh);
  std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>
      state_sampler = rrt_planner::ros_factory::createStateSampler<StateT>(
          nh, sampling_policy, state_space, collision_checker);

  switch (search_policy) {
    case rrt_planner::planner_core::planner::SearchPolicy::RRT:
      return rrt_planner::planner_core::planner_factory::createRRTPlanner(
          std::move(search_params), std::move(cost_scorer_params), state_space,
          state_connector, std::move(state_sampler), collision_checker);
    case rrt_planner::planner_core::planner::SearchPolicy::RRT_STAR: {
      rrt_planner::planner_core::nearest_neighbor_star_expander::
          NearestNeighborStarExpanderParams
              nearest_neighbor_star_expander_params =
                  rrt_planner::param_loader::loadNearestNeighborExpanderParams(
                      nh, collision_checker->getMapResolution());
      return rrt_planner::planner_core::planner_factory::createRRTStarPlanner(
          std::move(search_params), std::move(cost_scorer_params),
          std::move(nearest_neighbor_star_expander_params), state_space,
          state_connector, std::move(state_sampler), collision_checker);
    }
    case rrt_planner::planner_core::planner::SearchPolicy::BIDIRECTIONAL_RRT: {
      rrt_planner::planner_core::nearest_neighbor_tree_connector::
          NearestNeighborTreeConnectorParams
              nearest_neighbor_tree_connector_params = rrt_planner::
                  param_loader::loadNearestNeighborTreeConnectorParams(
                      nh, collision_checker->getMapResolution());
      return rrt_planner::planner_core::planner_factory::
          createBidirectionalRRTPlanner(
              std::move(search_params), std::move(cost_scorer_params),
              std::move(nearest_neighbor_tree_connector_params), state_space,
              state_connector, std::move(state_sampler), collision_checker);
    }
    case rrt_planner::planner_core::planner::SearchPolicy::
        BIDIRECTIONAL_RRT_STAR: {
      rrt_planner::planner_core::nearest_neighbor_tree_connector::
          NearestNeighborTreeConnectorParams
              nearest_neighbor_tree_connector_params = rrt_planner::
                  param_loader::loadNearestNeighborTreeConnectorParams(
                      nh, collision_checker->getMapResolution());
      rrt_planner::planner_core::nearest_neighbor_star_expander::
          NearestNeighborStarExpanderParams
              nearest_neighbor_star_expander_params =
                  rrt_planner::param_loader::loadNearestNeighborExpanderParams(
                      nh, collision_checker->getMapResolution());
      return rrt_planner::planner_core::planner_factory::
          createBidirectionalRRTStarPlanner(
              std::move(search_params), std::move(cost_scorer_params),
              std::move(nearest_neighbor_star_expander_params),
              std::move(nearest_neighbor_tree_connector_params), state_space,
              state_connector, std::move(state_sampler), collision_checker);
    }
    default:
      return nullptr;
  }
}
}  // namespace rrt_planner::ros_factory

#endif  // RRT_PLANNER__ROS_FACTORY__ROS_PLANNER_FACTORY_H_
