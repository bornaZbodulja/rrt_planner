/**
 * @file rrt_planner_2d.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/rrt_planner_2d.h"

#include <pluginlib/class_list_macros.h>

#include "nav_utils/geometry_utils.h"
#include "nav_utils/message_utils.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner2D, nav_core::BaseGlobalPlanner)

using namespace rrt_planner;

RRTPlanner2D::RRTPlanner2D() {
  costmap_ = nullptr;
  initialized_ = false;
}

RRTPlanner2D::RRTPlanner2D(std::string name,
                           costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

RRTPlanner2D::~RRTPlanner2D() { costmap_ = nullptr; }

void RRTPlanner2D::initialize(std::string name,
                              costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN(
        "This planner has already been initialized, you can't call it twice, "
        "doing nothing!");
    return;
  }

  costmap_ = costmap_ros->getCostmap();
  nh_ = ros::NodeHandle("~/" + name);

  // Initializing collision checker
  collision_checker_ = std::make_shared<CollisionCheckerT>(costmap_ros);

  // Loading planning parameters from param server
  LoadParams();

  // Initializing RRT* core planner
  rrt_star_ = std::make_unique<RRTStar<Node2D>>(motion_model_, search_info_,
                                                collision_checker_);

  WorldCoordinatesGetter world_coordinates_getter =
      [&, this](const auto& node_coordinates) -> geometry_msgs::Pose {
    return GetWorldCoordinates(node_coordinates.x, node_coordinates.y,
                               costmap_);
  };

  // Initializing visualization handler
  visualization_handler_ =
      std::make_shared<Visualization<Node2D>>(&nh_, world_coordinates_getter);

  ROS_INFO("Created global_planner rrt_planner/RRTPlanner2D.");

  initialized_ = true;
}

bool RRTPlanner2D::makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal,
                            PlanT& plan) {
  if (!initialized_) {
    ROS_WARN("Planner has not been initialized!");
    return false;
  }

  plan.clear();
  ClearVisualization();

  unsigned int start_mx, start_my;
  unsigned int goal_mx, goal_my;

  if (collision_checker_->PoseInCollision(start)) {
    ROS_WARN(
        "Start pose: (%f, %f, %f) in collision, returning planning failure",
        start.pose.position.x, start.pose.position.y,
        tf2::getYaw(start.pose.orientation));
    return false;
  }

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                            start_mx, start_my)) {
    ROS_WARN(
        "Unable to set start pose for planning, returning planning failure!");
    return false;
  }

  if (collision_checker_->PoseInCollision(goal)) {
    ROS_WARN("Goal pose: (%f, %f, %f) in collision, returning planning failure",
             goal.pose.position.x, goal.pose.position.y,
             tf2::getYaw(goal.pose.orientation));
    return false;
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx,
                            goal_my)) {
    ROS_WARN(
        "Unable to set goal pose for planning, returning planning failure!");
    return false;
  }

  ROS_INFO("Planning from start pose: (%f, %f, %f) to goal pose: (%f, %f, %f).",
           start.pose.position.x, start.pose.position.y,
           tf2::getYaw(start.pose.orientation), goal.pose.position.x,
           goal.pose.position.y, tf2::getYaw(goal.pose.orientation));

  auto path = CreatePath(start_mx, start_my, goal_mx, goal_my);
  const auto found_path = path.has_value();

  if (found_path) {
    plan = ProcessPath(path.value(), start, goal);
  }

  UpdateVisualization(plan, rrt_star_->GetStartTree(),
                      rrt_star_->GetGoalTree());
  PublishVisualization();

  if (!found_path) {
    ROS_WARN("RRT planner 2D unable to find plan, returning false.");
    return false;
  } else {
    ROS_INFO("RRT planner 2D successfully found a plan!");
    return true;
  }
}

void RRTPlanner2D::LoadParams() {
  double edge_length_double;
  nh_.param<double>("edge_length", edge_length_double, 0.2);
  // Scaling with costmap resolution
  search_info_.edge_length =
      std::ceil(edge_length_double / costmap_->getResolution());
  nh_.param<double>("target_bias", search_info_.target_bias, 0.05);
  nh_.param<double>("near_distance", search_info_.near_distance, 20.0);
  nh_.param<double>("cost_penalty", search_info_.cost_penalty, 2.0);
  nh_.param<bool>("rewire_tree", search_info_.rewire_tree, false);
  nh_.param<bool>("allow_unknown", search_info_.allow_unknown, false);
  nh_.param<int>("max_expansion_iterations",
                 search_info_.max_expansion_iterations, 100000);
  nh_.param<double>("max_planning_time", search_info_.max_planning_time, 5.0);
  nh_.param<double>("connect_trees_max_length",
                    search_info_.connect_trees_max_length, 10.0);
  // Scaling with costmap resolution
  search_info_.connect_trees_max_length /= costmap_->getResolution();

  int lethal_cost_int;
  nh_.param<int>("lethal_cost", lethal_cost_int, 253);
  search_info_.lethal_cost = static_cast<unsigned char>(lethal_cost_int);
  motion_model_ = MotionModel::TDM;
}

RRTPlanner2D::Plan2DT RRTPlanner2D::CreatePath(const unsigned int& start_mx,
                                               const unsigned int& start_my,
                                               const unsigned int& goal_mx,
                                               const unsigned int& goal_my) {
  rrt_star_->InitializeStateSpace(costmap_->getSizeInCellsX(),
                                  costmap_->getSizeInCellsY(), 1);

  rrt_star_->SetStart(start_mx, start_my, 1);
  rrt_star_->SetGoal(goal_mx, goal_my, 1);

  Node2D::CoordinatesVector path;

  const auto result = rrt_star_->CreatePath(path);

  if (!result) {
    return {};
  }

  return std::make_optional(path);
}

RRTPlanner2D::PlanT RRTPlanner2D::ProcessPath(
    Node2D::CoordinatesVector& path, const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal) {
  PlanT plan;
  plan.reserve(path.size());

  geometry_msgs::PoseStamped pose;

  pose.header = nav_utils::PrepareHeader("map");

  std::for_each(path.begin(), path.end(), [&](const auto& coordinates) {
    pose.pose = GetWorldCoordinates(coordinates.x, coordinates.y, costmap_);
    plan.push_back(pose);
  });

  plan.front().pose.orientation = start.pose.orientation;
  plan.back().pose.orientation = goal.pose.orientation;

  nav_utils::AssignPlanOrientation(plan);

  return plan;
}

void RRTPlanner2D::UpdateVisualization(const PlanT& plan,
                                       const TreeMsg& start_tree,
                                       const TreeMsg& goal_tree) {
  visualization_handler_->SetPathVisualization(plan);
  visualization_handler_->SetSearchTreeVisualization(start_tree, goal_tree);
}
