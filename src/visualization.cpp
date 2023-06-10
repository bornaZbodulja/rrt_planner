/**
 * @file visualization.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/visualization.h"

#include "nav_utils/message_utils.h"

using namespace rrt_planner;

template <typename NodeT>
Visualization<NodeT>::Visualization(
    ros::NodeHandle* nh, WorldCoordinatesGetter& world_coordinates_getter) {
  plan_pub_ = nh->advertise<nav_msgs::Path>("plan", 1, false);
  tree_pub_ =
      nh->advertise<visualization_msgs::MarkerArray>("search_tree", 1, false);
  world_coords_getter_ = world_coordinates_getter;
}

template <typename NodeT>
Visualization<NodeT>::~Visualization() {}

template <typename NodeT>
void Visualization<NodeT>::ClearVisualization() {
  ClearPath();
  ClearSearchTree();
  PublishVisualization();
}

template <typename NodeT>
void Visualization<NodeT>::SetPathVisualization(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  ClearPath();

  if (plan.empty()) {
    return;
  }

  path_.poses.reserve(plan.size());
  path_.poses.insert(path_.poses.begin(), plan.begin(), plan.end());
}

template <typename NodeT>
void Visualization<NodeT>::SetSearchTreeVisualization(
    const TreeMsg& start_tree, const TreeMsg& goal_tree) {
  AddTree(start_tree, START_TREE);
  AddTree(goal_tree, GOAL_TREE);
}

template <typename NodeT>
void Visualization<NodeT>::ClearPath() {
  // Setting header info
  path_.header = nav_utils::PrepareHeader("map");
  // Clearing poses
  path_.poses.clear();
}

template <typename NodeT>
void Visualization<NodeT>::ClearSearchTree() {
  for (auto& marker : search_tree_.markers) {
    marker.action = visualization_msgs::Marker::DELETE;
  }

  search_tree_.markers.clear();
}

// TODO: For now only 2D visualization, in future implement better solution for
// hybrid planner
template <typename NodeT>
void Visualization<NodeT>::AddTree(const TreeMsg& tree, const TreeId& id) {
  visualization_msgs::Marker marker;
  marker.header = nav_utils::PrepareHeader("map");
  marker.ns = "RRT_tree_" + std::to_string(id);
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color = TreeColorMapper(id);

  auto get_point = [this](const Coordinates& coordinates) {
    return world_coords_getter_(coordinates).position;
  };

  for (const auto& edge : tree) {
    marker.points.push_back(get_point(edge.first));
    marker.points.push_back(get_point(edge.second));
  }

  search_tree_.markers.push_back(marker);
}

// Instantiate algorithm for the supported template types
template class Visualization<Node2D>;
template class Visualization<NodeHybrid>;
