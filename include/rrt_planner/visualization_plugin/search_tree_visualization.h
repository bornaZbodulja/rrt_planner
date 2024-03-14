/**
 * @file search_tree_visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Search tree visualization utility
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <utility>
#include <vector>

namespace rrt_planner::visualization {
class SearchTreeVisualization {
 public:
  using PoseT = geometry_msgs::Pose;
  using EdgeT = std::vector<PoseT>;
  using TreeT = std::vector<EdgeT>;
  using MarkerT = visualization_msgs::Marker;
  using MarkerArrayT = visualization_msgs::MarkerArray;
  using ColorT = std_msgs::ColorRGBA;

  /**
   * @brief
   * @param nh
   * @param tree_identifier
   */
  SearchTreeVisualization(ros::NodeHandle* nh,
                          const std::string& tree_identifier,
                          const ColorT& tree_color);

  ~SearchTreeVisualization() = default;

  void publishVisualization() const { tree_pub_.publish(tree_); }

  void clearVisualization() {
    clearSearchTreeVisualization();
    publishVisualization();
  }

  void setTree(const TreeT& search_tree);

 private:
  MarkerT edgeToMarker(const std::string& edge_id, const EdgeT& edge);

  void clearSearchTreeVisualization() {
    std::for_each(tree_.markers.begin(), tree_.markers.end(),
                  [](MarkerT& marker) {
                    marker.action = visualization_msgs::Marker::DELETE;
                  });
  }

  // Tree id
  std::string tree_id_;
  // Search tree visualization publisher
  ros::Publisher tree_pub_;
  // Visualization of search tree
  MarkerArrayT tree_;
  // Color of the tree
  ColorT tree_color_;
};
}  // namespace rrt_planner::visualization

#endif  // RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_H_
