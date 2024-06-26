/**
 * @file cost_scorer_utilities.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Utility functions for working with cost scorers
 * @version 0.1
 * @date 2024-06-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__EXPANDER_UTILITIES__COST_SCORER_UTILITIES_H_
#define RRT_PLANNER__PLANNER_CORE__EXPANDER_UTILITIES__COST_SCORER_UTILITIES_H_

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/planner_entities/node.h"

namespace rrt_planner::planner_core::expander_utilities {
/**
 * @brief Computes accumulated cost of given child node for given parent node
 * @tparam StateT 
 * @param parent_node Given parent node pointer
 * @param child_node Given child node pointer
 * @param cost_scorer Cost scorer pointer
 * @return double 
 */
template <typename StateT>
double computeAccumulatedCostCost(
    const rrt_planner::planner_core::planner_entities::Node<StateT>* const
        parent_node,
    const rrt_planner::planner_core::planner_entities::Node<StateT>* const
        child_node,
    const rrt_planner::planner_core::cost_scorer::CostScorer<StateT>* const
        cost_scorer) {
  return (*cost_scorer)(parent_node->getState(), child_node->getState()) +
         parent_node->getAccumulatedCost();
}
}  // namespace rrt_planner::planner_core::expander_utilities

#endif  // RRT_PLANNER__PLANNER_CORE__EXPANDER_UTILITIES__COST_SCORER_UTILITIES_H_
