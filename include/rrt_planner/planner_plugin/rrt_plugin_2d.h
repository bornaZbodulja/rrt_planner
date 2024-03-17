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
#include "state_space/state_space_2d/space_2d.h"
#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_2d/state_space_2d.h"

namespace rrt_planner::planner_plugin {
class RRTPlugin2D : public nav_core::BaseGlobalPlanner {
 public:
  using State2D = state_space::state_space_2d::State2D;
  using State2DVector = std::vector<State2D>;
  using Space2D = state_space::state_space_2d::Space2D;
  using StateVector2D = std::vector<State2D>;
  using StateSpace2D = state_space::state_space_2d::StateSpace2D;
  using StateSpace2DPtr = std::shared_ptr<StateSpace2D>;
  using StateConnector2D =
      state_space::state_connector::StateConnector<State2D>;
  using StateConnector2DPtr = std::shared_ptr<StateConnector2D>;
  using RRTPlanner2D = rrt_planner::planner_core::planner::Planner<State2D>;
  using RRTPlanner2DPtr = std::unique_ptr<RRTPlanner2D>;
  using VisualizationT = rrt_planner::visualization::Visualization;
  using VisualizationPtr = std::unique_ptr<VisualizationT>;
  using CollisionCheckerT = nav_utils::CollisionChecker;
  using CollisionCheckerPtr = std::shared_ptr<CollisionCheckerT>;
  using PoseT = geometry_msgs::Pose;
  using PoseStampedT = geometry_msgs::PoseStamped;
  using PlanT = std::vector<PoseStampedT>;
  using Plan2DT = std::optional<StateVector2D>;
  using Tree2DT = std::vector<State2DVector>;
  using Tree2DVector = std::vector<Tree2DT>;
  using EdgeT = std::vector<PoseT>;
  using TreeT = std::vector<EdgeT>;
  using TreeVector = std::vector<TreeT>;

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
  bool makePlan(const PoseStampedT& start, const PoseStampedT& goal,
                PlanT& plan) override;

 private:
  /**
   * @brief
   * @param start
   * @param goal
   * @return Plan2DT
   */
  Plan2DT create2DPlan(const State2D& start, const State2D& goal);

  /**
   * @brief
   * @param plan_2d
   * @param start
   * @param goal
   * @param plan
   */
  void process2DPlan(const StateVector2D& plan_2d, const PoseStampedT& start,
                     const PoseStampedT& goal, PlanT& plan);

  void publishVisualization() const { visualization_->publishVisualization(); }

  void clearVisualization() { visualization_->clearVisualization(); }

  /**
   * @brief
   * @param plan
   */
  void updateVisualization(const PlanT& plan);

  /**
   * @brief
   * @param state_2d
   * @param pose
   */
  void state2DToPose(const State2D& state_2d, PoseT& pose);

  // Whether planner has already been initialized
  bool initialized_{false};
  // Private node handle
  ros::NodeHandle nh_;
  // Visualization plugin pointer
  VisualizationPtr visualization_;
  // RRT planner pointer
  RRTPlanner2DPtr planner_;
  // 2D state space pointer
  StateSpace2DPtr state_space_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
  // Map info
  MapInfo map_info_;
};
}  // namespace rrt_planner::planner_plugin

#endif
