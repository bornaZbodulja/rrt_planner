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
#include "rrt_planner/ros_factory/ros_planner_factory.h"
#include "rrt_planner/ros_factory/ros_state_connector_factory.h"
#include "rrt_planner/visualization_factory/visualization_factory.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::planner_plugin::RRTPluginHybrid,
                       nav_core::BaseGlobalPlanner)

using namespace rrt_planner::planner_plugin;
using namespace rrt_planner::param_loader;
using namespace rrt_planner::ros_factory;
using namespace rrt_planner::visualization_factory;

void RRTPluginHybrid::initialize(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("Planner %s has already been initialized!", name.c_str());
    return;
  }

  nh_ = ros::NodeHandle("~/" + name);

  collision_checker_ = std::make_shared<CollisionCheckerT>(costmap_ros);

  const auto costmap_resolution = costmap_ros->getCostmap()->getResolution();

  const auto space_hybrid = SpaceHybrid(collision_checker_->GetSizeX(),
                                        collision_checker_->GetSizeY(), 180);

  state_space_ = std::make_shared<StateSpaceHybrid>(space_hybrid);
  auto&& state_connector = ROSStateConnectorFactory::createHybridStateConnector(
      state_space_, collision_checker_, costmap_resolution, &nh_);

  auto search_policy = loadSearchPolicy(&nh_);
  auto sampling_policy = loadSamplingPolicy(&nh_);

  rrt_planner_ = ROSPlannerFactory::createPlanner<StateHybrid>(
      search_policy, sampling_policy, state_space_, std::move(state_connector),
      collision_checker_, costmap_resolution, &nh_);

  visualization_ =
      VisualizationFactory::createVisualization(search_policy, &nh_);

  initialized_ = true;
}

bool RRTPluginHybrid::makePlan(const PoseStampedT& start,
                               const PoseStampedT& goal, PlanT& plan) {
  if (!initialized_) {
    ROS_WARN("Planner has not been initialized");
    return false;
  }

  plan.clear();

  unsigned int start_mx, start_my, start_ang_bin;
  unsigned int goal_mx, goal_my, goal_ang_bin;

  if (collision_checker_->PoseInCollision(start)) {
    ROS_WARN(
        "Start pose: (%.3f, %.3f, %.3f) in collision, returning planning "
        "failure!",
        start.pose.position.x, start.pose.position.y,
        tf2::getYaw(start.pose.orientation));
    return false;
  }

  if (!collision_checker_->WorldToMap(
          start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    ROS_WARN(
        "Unable to set start pose for planning, returning planning failure!");
    return false;
  }

  if (collision_checker_->PoseInCollision(goal)) {
    ROS_WARN(
        "Goal pose: (%.3f, %.3f, %.3f) in collision, returning planning "
        "failure!",
        goal.pose.position.x, goal.pose.position.y,
        tf2::getYaw(goal.pose.orientation));
    return false;
  }

  if (!collision_checker_->WorldToMap(goal.pose.position.x,
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

  auto&& plan_hybrid = createHybridPlan(
      StateHybrid(static_cast<double>(start_mx), static_cast<double>(start_my),
                  static_cast<double>(start_ang_bin)),
      StateHybrid(static_cast<double>(goal_mx), static_cast<double>(goal_my),
                  static_cast<double>(goal_ang_bin)));

  const auto plan_found = plan_hybrid.has_value();

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

typename RRTPluginHybrid::PlanHybridT RRTPluginHybrid::createHybridPlan(
    StateHybrid start, StateHybrid goal) {
  rrt_planner_->initializeSearch();
  rrt_planner_->setStart(start);
  rrt_planner_->setGoal(goal);

  return rrt_planner_->createPath();
}

void RRTPluginHybrid::processHybridPlan(const StateVectorHybrid& plan_hybrid,
                                        PlanT& plan) {
  plan.reserve(plan_hybrid.size());

  PoseStampedT pose;
  pose.header = nav_utils::PrepareHeader("map");

  std::transform(plan_hybrid.cbegin(), plan_hybrid.cend(),
                 std::back_inserter(plan), [&](const auto& state_hybrid) {
                   stateHybridToPose(state_hybrid, pose.pose);
                   return pose;
                 });
}

void RRTPluginHybrid::updateVisualization(const PlanT& plan) {
  visualization_->updatePathVisualization(plan);
  TreeHybridVector&& tree_hybrid_vector = rrt_planner_->getSearchTrees();
  TreeVector tree_vector;
  tree_vector.reserve(tree_hybrid_vector.size());

  std::for_each(tree_hybrid_vector.cbegin(), tree_hybrid_vector.cend(),
                [&](const auto& tree_hybrid) {
                  PoseT child, parent;
                  TreeT tree;
                  tree.reserve(tree_hybrid.size());

                  std::transform(
                      tree_hybrid.cbegin(), tree_hybrid.cend(),
                      std::back_inserter(tree), [&](const auto& edge_hybrid) {
                        stateHybridToPose(edge_hybrid.first, child);
                        stateHybridToPose(edge_hybrid.second, parent);
                        return EdgeT{child, parent};
                      });

                  tree_vector.push_back(std::move(tree));
                });

  visualization_->updateSearchTreeVisualization(tree_vector);
}

void RRTPluginHybrid::stateHybridToPose(const StateHybrid& state_hybrid,
                                        PoseT& pose) {
  collision_checker_->MapToWorld(static_cast<unsigned int>(state_hybrid.x),
                                 static_cast<unsigned int>(state_hybrid.y),
                                 pose.position.x, pose.position.y);
  pose.orientation = tf2::toMsg(
      tf2::Quaternion{tf2::Vector3(0, 0, 1),
                      state_space_->getAngleFromBin(
                          static_cast<unsigned int>(state_hybrid.theta))});
}
