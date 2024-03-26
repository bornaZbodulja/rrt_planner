/**
 * @file rrt_plugin_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief ROS global planner plugin for 2D state space
 * @version 0.1
 * @date 2023-11-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_PLUGIN__RRT_PLUGIN_2D_H_
#define RRT_PLANNER__PLANNER_PLUGIN__RRT_PLUGIN_2D_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_utils/collision_checker.h>
#include <ros/node_handle.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_plugin/map_info.h"
#include "rrt_planner/visualization_plugin/visualization.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_connector_2d/state_connector_2d.h"
#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_2d/state_space_2d.h"

namespace rrt_planner::planner_plugin {
class RRTPlugin2D : public nav_core::BaseGlobalPlanner {
 public:
  using State2D = state_space::state_space_2d::State2D;

  RRTPlugin2D() = default;

  /**
   * @brief
   * @param name
   * @param costmap_ros
   */
  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief
   * @param start
   * @param goal
   * @param plan
   * @return true
   * @return false
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  /**
   * @brief
   * @param start
   * @param goal
   * @return std::optional<std::vector<State2D>>
   */
  std::optional<std::vector<State2D>> create2DPlan(const State2D& start,
                                                   const State2D& goal);

  /**
   * @brief
   * @param plan_2d
   * @param start
   * @param goal
   * @param plan
   */
  void process2DPlan(const std::vector<State2D>& plan_2d,
                     const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan);

  void publishVisualization() const { visualization_->publishVisualization(); }

  void clearVisualization() { visualization_->clearVisualization(); }

  /**
   * @brief
   * @param plan
   */
  void updateVisualization(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief
   * @param state_2d
   * @param pose
   */
  void state2DToPose(const State2D& state_2d, geometry_msgs::Pose& pose);

  // Whether planner has already been initialized
  bool initialized_{false};
  // Private node handle
  ros::NodeHandle nh_;
  // Visualization plugin pointer
  std::unique_ptr<rrt_planner::visualization::Visualization> visualization_;
  // RRT planner pointer
  std::unique_ptr<rrt_planner::planner_core::planner::Planner<State2D>>
      planner_;
  // 2D state space pointer
  std::shared_ptr<state_space::state_space_2d::StateSpace2D> state_space_;
  // Collision checker pointer
  std::shared_ptr<nav_utils::CollisionChecker> collision_checker_;
  // Map info
  MapInfo map_info_;
};
}  // namespace rrt_planner::planner_plugin

#endif
