/**
 * @file search_tree_visualization.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/visualization_plugin/search_tree_visualization.h"

#include "nav_utils/message_utils.h"

using namespace rrt_planner::visualization;

SearchTreeVisualization::SearchTreeVisualization(ros::NodeHandle* nh,
                                                 std::string tree_identifier,
                                                 const ColorT& tree_color)
    : tree_id_(tree_identifier),
      tree_pub_(
          nh->advertise<MarkerT>("search_tree/" + tree_identifier, 1, false)),
      tree_color_(tree_color) {}

void SearchTreeVisualization::setTree(const TreeT& search_tree) {
  MarkerT tree_marker;
  tree_marker.header = nav_utils::prepareHeader("map");
  tree_marker.ns = tree_id_;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.05;
  tree_marker.color = tree_color_;
  tree_marker.points.reserve(2 * search_tree.size());

  std::for_each(search_tree.cbegin(), search_tree.cend(),
                [&](const auto& edge) {
                  tree_marker.points.push_back(edge.first.position);
                  tree_marker.points.push_back(edge.second.position);
                });

  tree_ = std::move(tree_marker);
}
