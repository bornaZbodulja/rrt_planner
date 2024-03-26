/**
 * @file rrt_visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Visualization utility for basic RRT algorithm
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__RRT_VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__RRT_VISUALIZATION_H_

#include <geometry_msgs/Pose.h>
#include <ros/console.h>

#include <vector>

#include "rrt_planner/visualization_plugin/visualization.h"
#include "rrt_planner/visualization_plugin/visualization_utilities.h"

namespace rrt_planner::visualization {
class RRTVisualization : public Visualization {
 public:
  RRTVisualization(ros::NodeHandle* nh) : Visualization(nh) {
    this->tree_vis_->addTreeVisualization(nh, tree_id_, colorGreen());
  }

  ~RRTVisualization() override = default;

  void updateSearchTreeVisualization(
      const std::vector<std::vector<std::vector<geometry_msgs::Pose>>>& trees)
      override {
    if (trees.size() != 1) {
      ROS_WARN(
          "[RRTVisualization] Given number of search tree to visualize (%d) "
          "different than expected (1).",
          static_cast<int>(trees.size()));
      return;
    }

    tree_vis_->setTreeVisualization(tree_id_, trees[0]);
  }

 private:
  std::string tree_id_{"root_tree"};
};
}  // namespace rrt_planner::visualization

#endif  // RRT_PLANNER__VISUALIZATION_PLUGIN__RRT_VISUALIZATION_H_
