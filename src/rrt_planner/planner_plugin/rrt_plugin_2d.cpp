/**
 * @file rrt_plugin_2d.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/planner_plugin/rrt_plugin_2d.h"

#include <nav_utils/geometry_utils.h>
#include <nav_utils/message_utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include <algorithm>

#include "rrt_planner/param_loader/sampling_policy_loader.h"
#include "rrt_planner/param_loader/search_policy_loader.h"
#include "rrt_planner/ros_factory/ros_planner_factory.h"
#include "rrt_planner/ros_factory/ros_state_connector_factory.h"
#include "rrt_planner/visualization_factory/visualization_factory.h"
#include "state_space/state_space_2d/space_2d.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::planner_plugin::RRTPlugin2D,
                       nav_core::BaseGlobalPlanner)

using namespace rrt_planner::param_loader;
using namespace rrt_planner::ros_factory;
using namespace rrt_planner::visualization_factory;

namespace rrt_planner::planner_plugin {

void RRTPlugin2D::initialize(std::string name,
                             costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("Planner %s has already been initialized!", name.c_str());
    return;
  }

  nh_ = ros::NodeHandle("~/" + name);

  collision_checker_ = std::make_shared<CollisionCheckerT>(costmap_ros);

  const auto costmap_resolution = costmap_ros->getCostmap()->getResolution();

  const auto space_2d =
      Space2D(collision_checker_->getSizeX(), collision_checker_->getSizeY());

  state_space_ = std::make_shared<StateSpace2D>(space_2d);
  auto&& state_connector = ROSStateConnectorFactory::create2DStateConnector(
      collision_checker_, costmap_resolution, &nh_);

  auto search_policy = loadSearchPolicy(&nh_);
  auto sampling_policy = loadSamplingPolicy(&nh_);

  rrt_planner_ = ROSPlannerFactory::createPlanner<State2D>(
      search_policy, sampling_policy, state_space_, std::move(state_connector),
      collision_checker_, costmap_resolution, &nh_);

  visualization_ =
      VisualizationFactory::createVisualization(search_policy, &nh_);

  initialized_ = true;
}

bool RRTPlugin2D::makePlan(const PoseStampedT& start, const PoseStampedT& goal,
                           PlanT& plan) {
  if (!initialized_) {
    ROS_WARN("Planner has not been initialized");
    return false;
  }

  plan.clear();

  unsigned int start_mx, start_my;
  unsigned int goal_mx, goal_my;

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

  ROS_INFO("Planning from start pose: (%f, %f, %f) to goal pose: (%f, %f, %f).",
           start.pose.position.x, start.pose.position.y,
           tf2::getYaw(start.pose.orientation), goal.pose.position.x,
           goal.pose.position.y, tf2::getYaw(goal.pose.orientation));

  auto&& plan_2d = create2DPlan(
      State2D{static_cast<double>(start_mx), static_cast<double>(start_my)},
      State2D{static_cast<double>(goal_mx), static_cast<double>(goal_my)});

  const auto plan_found = plan_2d.has_value();

  if (plan_found) {
    process2DPlan(plan_2d.value(), start, goal, plan);
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

typename RRTPlugin2D::Plan2DT RRTPlugin2D::create2DPlan(State2D start,
                                                        State2D goal) {
  rrt_planner_->initializeSearch();
  rrt_planner_->setStart(start);
  rrt_planner_->setGoal(goal);

  return rrt_planner_->createPath();
}

void RRTPlugin2D::process2DPlan(const StateVector2D& plan_2d,
                                const PoseStampedT& start,
                                const PoseStampedT& goal, PlanT& plan) {
  plan.reserve(plan_2d.size());

  PoseStampedT pose;
  pose.header = nav_utils::prepareHeader("map");

  std::transform(plan_2d.cbegin(), plan_2d.cend(), std::back_inserter(plan),
                 [&](const auto& state_2d) {
                   state2DToPose(state_2d, pose.pose);
                   return pose;
                 });

  // Assign orientation to start and goal pose
  plan.front().pose.orientation = start.pose.orientation;
  plan.back().pose.orientation = goal.pose.orientation;

  nav_utils::assignPlanOrientation(plan);
}

void RRTPlugin2D::updateVisualization(const PlanT& plan) {
  visualization_->updatePathVisualization(plan);
  Tree2DVector&& tree_2d_vector = rrt_planner_->getSearchTrees();
  TreeVector tree_vector;
  tree_vector.reserve(tree_2d_vector.size());

  std::for_each(
      tree_2d_vector.cbegin(), tree_2d_vector.cend(), [&](const auto& tree_2d) {
        PoseT child, parent;
        TreeT tree;
        tree.reserve(tree_2d.size());

        std::transform(tree_2d.cbegin(), tree_2d.cend(),
                       std::back_inserter(tree), [&](const auto& edge_2d) {
                         state2DToPose(edge_2d.first, child);
                         state2DToPose(edge_2d.second, parent);
                         return EdgeT{child, parent};
                       });

        tree_vector.push_back(std::move(tree));
      });

  visualization_->updateSearchTreeVisualization(tree_vector);
}

void RRTPlugin2D::state2DToPose(const State2D& state_2d, PoseT& pose) {
  collision_checker_->mapToWorld(static_cast<unsigned int>(state_2d.x),
                                 static_cast<unsigned int>(state_2d.y),
                                 pose.position.x, pose.position.y);
}
}  // namespace rrt_planner::planner_plugin
