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

#include "rrt_planner/planner_core/search_policy.h"
#include "rrt_planner/visualization_plugin/bidirectional_rrt_visualization.h"
#include "rrt_planner/visualization_plugin/rrt_visualization.h"
#include "rrt_planner/visualization_plugin/visualization.h"

namespace rrt_planner::visualization_factory {
class VisualizationFactory {
 public:
  using VisualizationT = rrt_planner::visualization::Visualization;
  using VisualizationPtr = std::unique_ptr<VisualizationT>;
  using SearchPolicyT = rrt_planner::planner_core::SearchPolicy;
  using RRTVisualizationT = rrt_planner::visualization::RRTVisualization;
  using BidirectionalRRTVisualizationT =
      rrt_planner::visualization::BidirectionalRRTVisualization;

  VisualizationFactory() = delete;

  static VisualizationPtr createVisualization(SearchPolicyT search_policy,
                                              ros::NodeHandle* nh) {
    switch (search_policy) {
      case SearchPolicyT::RRT:
        return std::make_unique<RRTVisualizationT>(nh);
      case SearchPolicyT::BIDIRECTIONAL_RRT:
        return std::make_unique<BidirectionalRRTVisualizationT>(nh);
      case SearchPolicyT::RRT_STAR:
        return std::make_unique<RRTVisualizationT>(nh);
      case SearchPolicyT::BIDIRECTIONAL_RRT_STAR:
        return std::make_unique<BidirectionalRRTVisualizationT>(nh);
      default:
        return nullptr;
    }
  }
};
}  // namespace rrt_planner::visualization_factory

#endif
