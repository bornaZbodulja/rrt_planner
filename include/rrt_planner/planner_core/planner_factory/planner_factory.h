/**
 * @file planner_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planner factory
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__PLANNER_FACTORY_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__PLANNER_FACTORY_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/cost_scorer/cost_scorer_params.h"
#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander_params.h"
#include "rrt_planner/planner_core/nearest_neighbor_tree_connector/nearest_neighbor_tree_connector_params.h"
#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/planner_core/planner_factory/cost_scorer_factory.h"
#include "rrt_planner/planner_core/planner_factory/expander_factory.h"
#include "rrt_planner/planner_core/planner_factory/tree_connector_factory.h"
#include "rrt_planner/planner_core/planner_implementations/bidirectional_rrt.h"
#include "rrt_planner/planner_core/planner_implementations/rrt.h"
#include "rrt_planner/planner_core/tree_connector/tree_connector.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_factory {

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner::Planner<StateT>>
createRRTPlanner(
    rrt_planner::planner_core::planner::SearchParams&& search_params,
    rrt_planner::planner_core::cost_scorer::CostScorerParams&&
        cost_scorer_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
        state_sampler,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander = createNearestNeighborExpander<StateT>(
          createCostScorer<StateT>(std::move(cost_scorer_params), state_space,
                                   state_connector, collision_checker),
          state_space, state_connector);
  return std::make_unique<
      rrt_planner::planner_core::planner_implementations::RRT<StateT>>(
      rrt_planner::planner_core::planner::SearchPolicy::RRT,
      std::move(search_params), state_space, state_connector,
      std::move(expander), std::move(state_sampler));
}

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner::Planner<StateT>>
createRRTStarPlanner(
    rrt_planner::planner_core::planner::SearchParams&& search_params,
    rrt_planner::planner_core::cost_scorer::CostScorerParams&&
        cost_scorer_params,
    rrt_planner::planner_core::nearest_neighbor_star_expander::
        NearestNeighborStarExpanderParams&& star_expander_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
        state_sampler,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander = createNearestNeighborStarExpander<StateT>(
          std::move(star_expander_params),
          createCostScorer<StateT>(std::move(cost_scorer_params), state_space,
                                   state_connector, collision_checker),
          state_space, state_connector);
  return std::make_unique<
      rrt_planner::planner_core::planner_implementations::RRT<StateT>>(
      rrt_planner::planner_core::planner::SearchPolicy::RRT_STAR,
      std::move(search_params), state_space, state_connector,
      std::move(expander), std::move(state_sampler));
}

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner::Planner<StateT>>
createBidirectionalRRTPlanner(
    rrt_planner::planner_core::planner::SearchParams&& search_params,
    rrt_planner::planner_core::cost_scorer::CostScorerParams&&
        cost_scorer_params,
    rrt_planner::planner_core::nearest_neighbor_tree_connector::
        NearestNeighborTreeConnectorParams&& tree_connector_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
        state_sampler,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander = createNearestNeighborExpander<StateT>(
          createCostScorer<StateT>(std::move(cost_scorer_params), state_space,
                                   state_connector, collision_checker),
          state_space, state_connector);
  std::unique_ptr<
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>>
      tree_connector = createNearestNeighborTreeConnector<StateT>(
          std::move(tree_connector_params), state_connector);
  return std::make_unique<rrt_planner::planner_core::planner_implementations::
                              BidirectionalRRT<StateT>>(
      rrt_planner::planner_core::planner::SearchPolicy::BIDIRECTIONAL_RRT,
      std::move(search_params), state_space, state_connector,
      std::move(expander), std::move(tree_connector), std::move(state_sampler));
}

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner::Planner<StateT>>
createBidirectionalRRTStarPlanner(
    rrt_planner::planner_core::planner::SearchParams&& search_params,
    rrt_planner::planner_core::cost_scorer::CostScorerParams&&
        cost_scorer_params,
    rrt_planner::planner_core::nearest_neighbor_star_expander::
        NearestNeighborStarExpanderParams&& star_expander_params,
    rrt_planner::planner_core::nearest_neighbor_tree_connector::
        NearestNeighborTreeConnectorParams&& tree_connector_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    std::unique_ptr<state_space::state_sampler::StateSampler<StateT>>&&
        state_sampler,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
      expander = createNearestNeighborStarExpander<StateT>(
          std::move(star_expander_params),
          createCostScorer<StateT>(std::move(cost_scorer_params), state_space,
                                   state_connector, collision_checker),
          state_space, state_connector);
  std::unique_ptr<
      rrt_planner::planner_core::tree_connector::TreeConnector<StateT>>
      tree_connector = createNearestNeighborTreeConnector<StateT>(
          std::move(tree_connector_params), state_connector);
  return std::make_unique<rrt_planner::planner_core::planner_implementations::
                              BidirectionalRRT<StateT>>(
      rrt_planner::planner_core::planner::SearchPolicy::BIDIRECTIONAL_RRT_STAR,
      std::move(search_params), state_space, state_connector,
      std::move(expander), std::move(tree_connector), std::move(state_sampler));
}
}  // namespace rrt_planner::planner_core::planner_factory

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__PLANNER_FACTORY_H_
