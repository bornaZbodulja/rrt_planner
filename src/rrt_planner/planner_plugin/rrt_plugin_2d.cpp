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

namespace rrt_planner::planner_plugin {

void RRTPlugin2D::initialize(std::string name,
                             costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("Planner %s has already been initialized!", name.c_str());
    return;
  }

  nh_ = ros::NodeHandle("~/" + name);

  collision_checker_ = std::make_shared<CollisionCheckerT>(costmap_ros);

  map_info_ =
      MapInfo(costmap_ros->getCostmap()->getOriginX(),
              costmap_ros->getCostmap()->getOriginY(),
              static_cast<double>(costmap_ros->getCostmap()->getSizeInCellsX()),
              static_cast<double>(costmap_ros->getCostmap()->getSizeInCellsY()),
              costmap_ros->getCostmap()->getResolution());

  Space2D space_2d = Space2D(map_info_.size.x, map_info_.size.y);

  state_space_ = std::make_shared<StateSpace2D>(std::move(space_2d));

  auto search_policy = rrt_planner::param_loader::loadSearchPolicy(&nh_);
  auto sampling_policy = rrt_planner::param_loader::loadSamplingPolicy(&nh_);
  StateConnector2DPtr state_connector =
      rrt_planner::ros_factory::create2DStateConnector(&nh_,
                                                       collision_checker_);

  planner_ = rrt_planner::ros_factory::createPlanner<State2D>(
      &nh_, search_policy, sampling_policy, state_space_, state_connector,
      collision_checker_);

  visualization_ = rrt_planner::visualization_factory::createVisualization(
      &nh_, search_policy);

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

  auto plan_2d = create2DPlan(
      State2D{static_cast<double>(start_mx), static_cast<double>(start_my)},
      State2D{static_cast<double>(goal_mx), static_cast<double>(goal_my)});

  bool plan_found = plan_2d.has_value();

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

RRTPlugin2D::Plan2DT RRTPlugin2D::create2DPlan(const State2D& start,
                                               const State2D& goal) {
  planner_->initializeSearch();
  planner_->setStart(start);
  planner_->setGoal(goal);

  return planner_->createPath();
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
  Tree2DVector tree_2d_vector = planner_->getTrees();
  TreeVector tree_vector;
  tree_vector.reserve(tree_2d_vector.size());

  std::transform(tree_2d_vector.cbegin(), tree_2d_vector.cend(),
                 std::back_inserter(tree_vector), [&](const Tree2DT& tree_2d) {
                   TreeT tree;
                   tree.reserve(tree_2d.size());

                   std::transform(tree_2d.cbegin(), tree_2d.cend(),
                                  std::back_inserter(tree),
                                  [&](const State2DVector& edge_2d) {
                                    EdgeT edge;
                                    edge.reserve(edge_2d.size());

                                    std::transform(
                                        edge_2d.cbegin(), edge_2d.cend(),
                                        std::back_inserter(edge),
                                        [&](const State2D& state_2d) {
                                          PoseT pose;
                                          state2DToPose(state_2d, pose);
                                          return pose;
                                        });
                                    return edge;
                                  });
                   return tree;
                 });

  visualization_->updateSearchTreeVisualization(tree_vector);
}

void RRTPlugin2D::state2DToPose(const State2D& state_2d, PoseT& pose) {
  pose.position.x =
      map_info_.origin.x + map_info_.resolution * (state_2d.x + 0.5);
  pose.position.y =
      map_info_.origin.y + map_info_.resolution * (state_2d.y + 0.5);
}
}  // namespace rrt_planner::planner_plugin
