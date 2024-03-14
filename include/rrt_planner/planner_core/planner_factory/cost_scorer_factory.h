/**
 * @file cost_scorer_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Cost scorer factory
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__COST_SCORER_FACTORY_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__COST_SCORER_FACTORY_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/cost_scorer/cost_scorer_params.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_factory {

template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>
createCostScorer(
    rrt_planner::planner_core::cost_scorer::CostScorerParams&&
        cost_scorer_params,
    const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
    const std::shared_ptr<state_space::state_connector::StateConnector<StateT>>&
        state_connector,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  return std::make_unique<
      rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>(
      std::move(cost_scorer_params), state_space, state_connector,
      collision_checker);
}
}  // namespace rrt_planner::planner_core::planner_factory

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_FACTORY__COST_SCORER_FACTORY_H_
