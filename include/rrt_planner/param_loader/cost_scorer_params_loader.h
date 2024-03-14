/**
 * @file cost_scorer_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__COST_SCORER_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__COST_SCORER_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "rrt_planner/planner_core/cost_scorer/cost_scorer_params.h"

namespace rrt_planner::param_loader {

inline rrt_planner::planner_core::cost_scorer::CostScorerParams
loadCostScorerParams(ros::NodeHandle* nh) {
  rrt_planner::planner_core::cost_scorer::CostScorerParams cost_scorer_params;
  loadParam<double>(nh, "cost_penalty", cost_scorer_params.cost_penalty);
  loadParam<double>(nh, "traversal_penalty",
                    cost_scorer_params.traversal_penalty);
  return cost_scorer_params;
}
}  // namespace rrt_planner::param_loader

#endif  // RRT_PLANNER__PARAM_LOADER__COST_SCORER_PARAMS_LOADER_H_
