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

#include <memory>
#include <vector>

#include "rrt_planner/visualization_plugin/search_tree_visualization_collection.h"
#include "rrt_planner/visualization_plugin/tree_id.h"
#include "rrt_planner/visualization_plugin/visualization.h"
#include "rrt_planner/visualization_plugin/visualization_utilities.h"

namespace rrt_planner::visualization {
class BidirectionalRRTVisualization : public Visualization {
 public:
  BidirectionalRRTVisualization(ros::NodeHandle* nh)
      : Visualization(nh),
        tree_vis_(std::make_unique<SearchTreeVisualizationCollection>()) {
    tree_vis_->addTreeVisualization(nh, TreeId::ROOT_TREE,
                                    treeColorMapper(TreeId::ROOT_TREE));
    tree_vis_->addTreeVisualization(nh, TreeId::TARGET_TREE,
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

    tree_vis_->setTreeVisualization(TreeId::ROOT_TREE, trees[0]);
    tree_vis_->setTreeVisualization(TreeId::TARGET_TREE, trees[1]);
  }

  void publishVisualization() const override {
    tree_vis_->publishVisualization();
    Visualization::publishVisualization();
  }

  void clearVisualization() override {
    tree_vis_->clearVisualization();
    Visualization::clearVisualization();
  }

 private:
  std_msgs::ColorRGBA treeColorMapper(TreeId id) {
    switch (id) {
      case TreeId::ROOT_TREE:
        return colorGreen();
      case TreeId::TARGET_TREE:
        return colorRed();
    }

    __builtin_unreachable();
  }

  // Search tree visualization utility
  std::unique_ptr<SearchTreeVisualizationCollection> tree_vis_;
};
}  // namespace rrt_planner::visualization

#endif
