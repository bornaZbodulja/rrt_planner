/**
 * @file expander_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Expander factory
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__EXPANDER_FACTORY_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__EXPANDER_FACTORY_H_

#include <memory>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_expander/nearest_neighbor_expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander_params.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_factory {

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
createNearestNeighborExpander(
    std::unique_ptr<rrt_planner::planner_core::cost_scorer::CostScorer<
        StateT>>&& cost_scorer,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector) {
  return std::make_unique<rrt_planner::planner_core::nearest_neighbor_expander::
                              NearestNeighborExpander<StateT>>(
      std::move(cost_scorer), state_space, state_connector);
}

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::expander::Expander<StateT>>
createNearestNeighborStarExpander(
    rrt_planner::planner_core::nearest_neighbor_star_expander::
        NearestNeighborStarExpanderParams&& star_expander_params,
    std::unique_ptr<
        rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>&&
        cost_scorer,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector) {
  return std::make_unique<
      rrt_planner::planner_core::nearest_neighbor_star_expander::
          NearestNeighborStarExpander<StateT>>(std::move(star_expander_params),
                                               std::move(cost_scorer),
                                               state_space, state_connector);
}
}  // namespace rrt_planner::planner_core::planner_factory

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__EXPANDER_FACTORY_H_
