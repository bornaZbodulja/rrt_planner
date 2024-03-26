/**
 * @file visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Visualization utility
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>

#include <memory>
#include <vector>

#include "rrt_planner/visualization_plugin/path_visualization.h"
#include "rrt_planner/visualization_plugin/search_tree_visualization.h"
#include "rrt_planner/visualization_plugin/search_tree_visualization_collection.h"

namespace rrt_planner::visualization {
class Visualization {
 public:
  Visualization(ros::NodeHandle* nh)
      : tree_vis_(std::make_unique<SearchTreeVisualizationCollection>()),
        path_vis_(std::make_unique<PathVisualization>(nh)) {}

  virtual ~Visualization() = default;

  void updatePathVisualization(
      const std::vector<geometry_msgs::PoseStamped>& plan) {
    path_vis_->setPath(plan);
  }

  /**
   * @brief
   * @param trees
   */
  virtual void updateSearchTreeVisualization(
      const std::vector<std::vector<std::vector<geometry_msgs::Pose>>>&
          trees) = 0;

  void publishVisualization() const {
    path_vis_->publishVisualization();
    tree_vis_->publishVisualization();
  }

  void clearVisualization() {
    path_vis_->clearVisualization();
    tree_vis_->clearVisualization();
  }

 protected:
  // Search tree visualization utility
  std::unique_ptr<SearchTreeVisualizationCollection> tree_vis_;

 private:
  // Path visualization utility
  std::unique_ptr<PathVisualization> path_vis_;
};
}  // namespace rrt_planner::visualization

#endif
