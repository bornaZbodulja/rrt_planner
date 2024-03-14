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

namespace rrt_planner::visualization {

SearchTreeVisualization::SearchTreeVisualization(
    ros::NodeHandle* nh, const std::string& tree_identifier,
    const ColorT& tree_color)
    : tree_id_(tree_identifier),
      tree_pub_(nh->advertise<MarkerArrayT>("search_tree/" + tree_identifier, 1,
                                            false)),
      tree_color_(tree_color) {}

void SearchTreeVisualization::setTree(const TreeT& search_tree) {
  tree_.markers.clear();
  tree_.markers.reserve(search_tree.size());

  unsigned int idx = 0;
  std::transform(search_tree.cbegin(), search_tree.cend(),
                 std::back_inserter(tree_.markers),
                 [this, &idx](const EdgeT& edge) {
                   return edgeToMarker("edge_" + std::to_string(idx++), edge);
                 });
}

SearchTreeVisualization::MarkerT SearchTreeVisualization::edgeToMarker(
    const std::string& edge_id, const EdgeT& edge) {
  MarkerT edge_marker;
  edge_marker.header = nav_utils::prepareHeader("map");
  edge_marker.ns = tree_id_ + "/" + edge_id;
  edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.05;
  edge_marker.color = tree_color_;
  edge_marker.points.reserve(edge.size());

  std::transform(edge.cbegin(), edge.cend(),
                 std::back_inserter(edge_marker.points),
                 [](const PoseT& pose) { return pose.position; });
  return edge_marker;
}
}  // namespace rrt_planner::visualization
