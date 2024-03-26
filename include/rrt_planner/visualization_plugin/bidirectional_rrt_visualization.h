/**
 * @file bidirectional_rrt_visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Visualization utility for bidirectional RRT algorithm
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__BIDIRECTIONAL_RRT_VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__BIDIRECTIONAL_RRT_VISUALIZATION_H_

#include <geometry_msgs/Pose.h>
#include <ros/console.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>

#include "rrt_planner/visualization_plugin/visualization.h"
#include "rrt_planner/visualization_plugin/visualization_utilities.h"

namespace rrt_planner::visualization {
class BidirectionalRRTVisualization : public Visualization {
 public:
  enum class TreeId { ROOT_TREE = 0, TARGET_TREE = 1 };

  BidirectionalRRTVisualization(ros::NodeHandle* nh) : Visualization(nh) {
    this->tree_vis_->addTreeVisualization(nh, treeIdToString(TreeId::ROOT_TREE),
                                          treeColorMapper(TreeId::ROOT_TREE));
    this->tree_vis_->addTreeVisualization(nh,
                                          treeIdToString(TreeId::TARGET_TREE),
                                          treeColorMapper(TreeId::TARGET_TREE));
  }

  ~BidirectionalRRTVisualization() override = default;

  void updateSearchTreeVisualization(
      const std::vector<std::vector<std::vector<geometry_msgs::Pose>>>& trees)
      override {
    if (trees.size() != 2) {
      ROS_WARN(
          "[BidirectionalRRTVisualization] Given number of search trees to "
          "visualize (%d) different than expected (2).",
          static_cast<int>(trees.size()));
      return;
    }

    this->tree_vis_->setTreeVisualization(treeIdToString(TreeId::ROOT_TREE),
                                          trees[0]);
    this->tree_vis_->setTreeVisualization(treeIdToString(TreeId::TARGET_TREE),
                                          trees[1]);
  }

 private:
  std::string treeIdToString(TreeId id) {
    switch (id) {
      case TreeId::ROOT_TREE:
        return "root_tree";
      case TreeId::TARGET_TREE:
        return "target_tree";
    }

    __builtin_unreachable();
  }

  std_msgs::ColorRGBA treeColorMapper(TreeId id) {
    switch (id) {
      case TreeId::ROOT_TREE:
        return colorGreen();
      case TreeId::TARGET_TREE:
        return colorRed();
    }

    __builtin_unreachable();
  }
};
}  // namespace rrt_planner::visualization

#endif
