/**
 * @file visualization_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Visualization factory
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_FACTORY_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_FACTORY_H_

#include <ros/node_handle.h>

#include <memory>

#include "rrt_planner/planner_core/planner_implementations/search_policy.h"
#include "rrt_planner/visualization_plugin/bidirectional_rrt_visualization.h"
#include "rrt_planner/visualization_plugin/rrt_visualization.h"
#include "rrt_planner/visualization_plugin/visualization.h"

namespace rrt_planner::visualization_factory {

inline std::unique_ptr<rrt_planner::visualization::Visualization>
createVisualization(
    ros::NodeHandle* nh,
    rrt_planner::planner_core::planner_implementations::SearchPolicy
        search_policy) {
  switch (search_policy) {
    case rrt_planner::planner_core::planner_implementations::SearchPolicy::RRT:
      return std::make_unique<rrt_planner::visualization::RRTVisualization>(nh);
    case rrt_planner::planner_core::planner_implementations::SearchPolicy::
        BIDIRECTIONAL_RRT:
      return std::make_unique<
          rrt_planner::visualization::BidirectionalRRTVisualization>(nh);
    case rrt_planner::planner_core::planner_implementations::SearchPolicy::
        RRT_STAR:
      return std::make_unique<rrt_planner::visualization::RRTVisualization>(nh);
    case rrt_planner::planner_core::planner_implementations::SearchPolicy::
        BIDIRECTIONAL_RRT_STAR:
      return std::make_unique<
          rrt_planner::visualization::BidirectionalRRTVisualization>(nh);
    default:
      return nullptr;
  }
}
}  // namespace rrt_planner::visualization_factory

#endif  // RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_FACTORY_H_
