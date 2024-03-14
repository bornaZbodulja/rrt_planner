/**
 * @file cost_scorer_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Cost scorer parameters
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__COST_SCORER__COST_SCORER_PARAMS_H_
#define RRT_PLANNER__PLANNER_CORE__COST_SCORER__COST_SCORER_PARAMS_H_

namespace rrt_planner::planner_core::cost_scorer {
/**
 * @brief Representation of cost scorer parameters
 */
struct CostScorerParams {
  // Penalty to apply to high cost areas
  double cost_penalty{0.0};
  // Penalty to apply to traversal distance between parent and child node
  double traversal_penalty{0.0};
};
}  // namespace rrt_planner::planner_core::cost_scorer

#endif
