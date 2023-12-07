/**
 * @file path_visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Path visualization utility
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__PATH_VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__PATH_VISUALIZATION_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <vector>

#include "nav_utils/message_utils.h"

namespace rrt_planner::visualization {
class PathVisualization {
 public:
  using PoseT = geometry_msgs::PoseStamped;
  using PathT = nav_msgs::Path;
  using PlanT = std::vector<PoseT>;

  /**
   * @brief
   * @param nh
   */
  PathVisualization(ros::NodeHandle* nh)
      : path_pub_(nh->advertise<PathT>("path", 1, false)) {}

  ~PathVisualization() = default;

  inline void publishVisualization() const { path_pub_.publish(path_); }

  inline void clearVisualization() {
    clearPath();
    publishVisualization();
  }

  void setPath(const PlanT& plan) {
    clearPath();
    if (plan.empty()) {
      return;
    }

    path_.poses.reserve(plan.size());
    path_.poses.insert(path_.poses.begin(), plan.begin(), plan.end());
  }

 private:
  void clearPath() {
    // Setting header info
    path_.header = nav_utils::PrepareHeader("map");
    // Clearing poses
    path_.poses.clear();
  }

  // Path publisher
  ros::Publisher path_pub_;
  // Visualization of planned path
  PathT path_;
};
}  // namespace rrt_planner::visualization

#endif
