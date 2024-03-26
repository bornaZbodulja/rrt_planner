/**
 * @file rrt_plugin_hybrid.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/planner_plugin/rrt_plugin_hybrid.h"

#include <nav_utils/geometry_utils.h>
#include <nav_utils/message_utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <tf2/utils.h>

#include <algorithm>

#include "rrt_planner/param_loader/sampling_policy_loader.h"
#include "rrt_planner/param_loader/search_policy_loader.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/ros_factory/ros_planner_factory.h"
#include "rrt_planner/ros_factory/ros_state_connector_factory.h"
#include "rrt_planner/visualization_factory/visualization_factory.h"
#include "state_space/state_sampler/sampling_policy.h"
#include "state_space/state_space_hybrid/space_hybrid.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::planner_plugin::RRTPluginHybrid,
                       nav_core::BaseGlobalPlanner)

namespace rrt_planner::planner_plugin {

void RRTPluginHybrid::initialize(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("Planner %s has already been initialized!", name.c_str());
    return;
  }

  nh_ = ros::NodeHandle("~/" + name);

  collision_checker_ =
      std::make_shared<nav_utils::CollisionChecker>(costmap_ros);

  map_info_ =
      MapInfo(costmap_ros->getCostmap()->getOriginX(),
              costmap_ros->getCostmap()->getOriginY(),
              static_cast<double>(costmap_ros->getCostmap()->getSizeInCellsX()),
              static_cast<double>(costmap_ros->getCostmap()->getSizeInCellsY()),
              costmap_ros->getCostmap()->getResolution());

  // TODO: Expose bin size as parameter (180)
  state_space::state_space_hybrid::SpaceHybrid space_hybrid =
      state_space::state_space_hybrid::SpaceHybrid(map_info_.size.x,
                                                   map_info_.size.y, 180);

  state_space_ =
      std::make_shared<state_space::state_space_hybrid::StateSpaceHybrid>(
          std::move(space_hybrid));
  std::shared_ptr<state_space::state_connector::StateConnector<StateHybrid>>
      state_connector = rrt_planner::ros_factory::createHybridStateConnector(
          &nh_, state_space_, collision_checker_);

  rrt_planner::planner_core::planner::SearchPolicy search_policy =
      rrt_planner::param_loader::loadSearchPolicy(&nh_);
  state_space::state_sampler::SamplingPolicy sampling_policy =
      rrt_planner::param_loader::loadSamplingPolicy(&nh_);

  planner_ = rrt_planner::ros_factory::createPlanner<StateHybrid>(
      &nh_, search_policy, sampling_policy, state_space_, state_connector,
      collision_checker_);

  visualization_ = rrt_planner::visualization_factory::createVisualization(
      &nh_, search_policy);

  initialized_ = true;
}

bool RRTPluginHybrid::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_WARN("Planner has not been initialized");
    return false;
  }

  plan.clear();

  unsigned int start_mx, start_my, start_ang_bin;
  unsigned int goal_mx, goal_my, goal_ang_bin;

  if (collision_checker_->poseInCollision(start)) {
    ROS_WARN(
        "Start pose: (%.3f, %.3f, %.3f) in collision, returning planning "
        "failure!",
        start.pose.position.x, start.pose.position.y,
        tf2::getYaw(start.pose.orientation));
    return false;
  }

  if (!collision_checker_->worldToMap(
          start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    ROS_WARN(
        "Unable to set start pose for planning, returning planning failure!");
    return false;
  }

  if (collision_checker_->poseInCollision(goal)) {
    ROS_WARN(
        "Goal pose: (%.3f, %.3f, %.3f) in collision, returning planning "
        "failure!",
        goal.pose.position.x, goal.pose.position.y,
        tf2::getYaw(goal.pose.orientation));
    return false;
  }

  if (!collision_checker_->worldToMap(goal.pose.position.x,
                                      goal.pose.position.y, goal_mx, goal_my)) {
    ROS_WARN(
        "Unable to set goal pose for planning, returning planning failure!");
    return false;
  }

  start_ang_bin =
      state_space_->getClosestAngularBin(tf2::getYaw(start.pose.orientation));
  goal_ang_bin =
      state_space_->getClosestAngularBin(tf2::getYaw(goal.pose.orientation));

  ROS_INFO("Planning from start pose: (%f, %f, %f) to goal pose: (%f, %f, %f).",
           start.pose.position.x, start.pose.position.y,
           tf2::getYaw(start.pose.orientation), goal.pose.position.x,
           goal.pose.position.y, tf2::getYaw(goal.pose.orientation));

  std::optional<std::vector<StateHybrid>> plan_hybrid = createHybridPlan(
      StateHybrid(static_cast<double>(start_mx), static_cast<double>(start_my),
                  static_cast<double>(start_ang_bin)),
      StateHybrid(static_cast<double>(goal_mx), static_cast<double>(goal_my),
                  static_cast<double>(goal_ang_bin)));

  bool plan_found = plan_hybrid.has_value();

  if (plan_found) {
    processHybridPlan(plan_hybrid.value(), plan);
  }

  clearVisualization();
  updateVisualization(plan);
  publishVisualization();

  if (!plan_found) {
    ROS_WARN("Planner unable to find plan, returning false.");
    return false;
  } else {
    ROS_INFO("Planner successfully found a plan!");
    return true;
  }
}

std::optional<std::vector<RRTPluginHybrid::StateHybrid>>
RRTPluginHybrid::createHybridPlan(const StateHybrid& start,
                                  const StateHybrid& goal) {
  planner_->initializeSearch();
  planner_->setStart(start);
  planner_->setGoal(goal);

  return planner_->createPath();
}

void RRTPluginHybrid::processHybridPlan(
    const std::vector<StateHybrid>& plan_hybrid,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.reserve(plan_hybrid.size());

  geometry_msgs::PoseStamped pose;
  pose.header = nav_utils::prepareHeader("map");

  std::transform(plan_hybrid.cbegin(), plan_hybrid.cend(),
                 std::back_inserter(plan),
                 [&](const StateHybrid& state_hybrid) {
                   stateHybridToPose(state_hybrid, pose.pose);
                   return pose;
                 });
}

void RRTPluginHybrid::updateVisualization(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  visualization_->updatePathVisualization(plan);
  std::vector<std::vector<std::vector<StateHybrid>>> tree_hybrid_vector =
      planner_->getTrees();
  std::vector<std::vector<std::vector<geometry_msgs::Pose>>> tree_vector;
  tree_vector.reserve(tree_hybrid_vector.size());

  std::transform(
      tree_hybrid_vector.cbegin(), tree_hybrid_vector.cend(),
      std::back_inserter(tree_vector),
      [&](const std::vector<std::vector<StateHybrid>>& tree_hybrid) {
        std::vector<std::vector<geometry_msgs::Pose>> tree;
        tree.reserve(tree_hybrid.size());

        std::transform(
            tree_hybrid.cbegin(), tree_hybrid.cend(), std::back_inserter(tree),
            [&](const std::vector<StateHybrid>& edge_hybrid) {
              std::vector<geometry_msgs::Pose> edge;
              edge.reserve(edge_hybrid.size());

              std::transform(edge_hybrid.cbegin(), edge_hybrid.cend(),
                             std::back_inserter(edge),
                             [&](const StateHybrid& state_hybrid) {
                               geometry_msgs::Pose pose;
                               stateHybridToPose(state_hybrid, pose);
                               return pose;
                             });
              return edge;
            });
        return tree;
      });

  visualization_->updateSearchTreeVisualization(tree_vector);
}

void RRTPluginHybrid::stateHybridToPose(const StateHybrid& state_hybrid,
                                        geometry_msgs::Pose& pose) {
  pose.position.x =
      map_info_.origin.x + map_info_.resolution * (state_hybrid.x + 0.5);
  pose.position.y =
      map_info_.origin.y + map_info_.resolution * (state_hybrid.y + 0.5);
  pose.orientation = tf2::toMsg(
      tf2::Quaternion{tf2::Vector3(0, 0, 1),
                      state_space_->getAngleFromBin(
                          static_cast<unsigned int>(state_hybrid.theta))});
}
}  // namespace rrt_planner::planner_plugin
