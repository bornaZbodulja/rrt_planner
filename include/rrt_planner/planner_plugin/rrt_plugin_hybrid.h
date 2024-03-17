/**
 * @file rrt_plugin_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief ROS global planner plugin for hybrid state space
 * @version 0.1
 * @date 2023-11-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_PLUGIN__RRT_PLUGIN_HYBRID_H_
#define RRT_PLANNER__PLANNER_PLUGIN__RRT_PLUGIN_HYBRID_H_

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
#include "state_space/state_space_hybrid/space_hybrid.h"
#include "state_space/state_space_hybrid/state_hybrid.h"
#include "state_space/state_space_hybrid/state_space_hybrid.h"

namespace rrt_planner::planner_plugin {
class RRTPluginHybrid : public nav_core::BaseGlobalPlanner {
 public:
  using StateHybrid = state_space::state_space_hybrid::StateHybrid;
  using StateHybridVector = std::vector<StateHybrid>;
  using SpaceHybrid = state_space::state_space_hybrid::SpaceHybrid;
  using StateVectorHybrid = std::vector<StateHybrid>;
  using StateSpaceHybrid = state_space::state_space_hybrid::StateSpaceHybrid;
  using StateSpaceHybridPtr = std::shared_ptr<StateSpaceHybrid>;
  using StateConnectorHybrid =
      state_space::state_connector::StateConnector<StateHybrid>;
  using StateConnectorHybridPtr = std::shared_ptr<StateConnectorHybrid>;
  using RRTPlannerHybrid =
      rrt_planner::planner_core::planner::Planner<StateHybrid>;
  using RRTPlannerHybridPtr = std::unique_ptr<RRTPlannerHybrid>;
  using VisualizationT = rrt_planner::visualization::Visualization;
  using VisualizationPtr = std::unique_ptr<VisualizationT>;
  using CollisionCheckerT = nav_utils::CollisionChecker;
  using CollisionCheckerPtr = std::shared_ptr<CollisionCheckerT>;
  using PoseT = geometry_msgs::Pose;
  using PoseStampedT = geometry_msgs::PoseStamped;
  using PlanT = std::vector<PoseStampedT>;
  using PlanHybridT = std::optional<StateVectorHybrid>;
  using TreeHybridT = std::vector<StateHybridVector>;
  using TreeHybridVector = std::vector<TreeHybridT>;
  using EdgeT = std::vector<PoseT>;
  using TreeT = std::vector<EdgeT>;
  using TreeVector = std::vector<TreeT>;

  RRTPluginHybrid() = default;

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
   * @return PlanHybridT
   */
  PlanHybridT createHybridPlan(const StateHybrid& start,
                               const StateHybrid& goal);

  /**
   * @brief
   * @param plan_hybrid
   * @param plan
   */
  void processHybridPlan(const StateVectorHybrid& plan_hybrid, PlanT& plan);

  void publishVisualization() const { visualization_->publishVisualization(); }

  void clearVisualization() { visualization_->clearVisualization(); }

  /**
   * @brief
   * @param plan
   */
  void updateVisualization(const PlanT& plan);

  /**
   * @brief
   * @param state_hybrid
   * @param pose
   */
  void stateHybridToPose(const StateHybrid& state_hybrid, PoseT& pose);

  // Whether planner has already been initialized
  bool initialized_{false};
  // Private node handle
  ros::NodeHandle nh_;
  // Visualization plugin pointer
  VisualizationPtr visualization_;
  // RRT planner pointer
  RRTPlannerHybridPtr planner_;
  // Hybrid state space pointer
  StateSpaceHybridPtr state_space_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
  // Map info
  MapInfo map_info_;
};
}  // namespace rrt_planner::planner_plugin

#endif  // RRT_PLANNER__PLANNER_PLUGIN__RRT_PLUGIN_HYBRID_H_
