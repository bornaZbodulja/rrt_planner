/**
 * @file rrt_planner_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT Hybrid planner plugin
 * @version 0.1
 * @date 2023-06-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__RRT_PLANNER_HYBRID_H_
#define RRT_PLANNER__RRT_PLANNER_HYBRID_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/node_handle.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

#include "rrt_planner/constants.h"
#include "rrt_planner/rrt_star.h"
#include "rrt_planner/types.h"
#include "rrt_planner/utils.h"
#include "rrt_planner/visualization.h"

namespace rrt_planner {

/**
 * @brief RRT Planner Hybrid global planner plugin
 */
class RRTPlannerHybrid : public nav_core::BaseGlobalPlanner {
 public:
  using WorldCoordinatesGetter =
      std::function<geometry_msgs::Pose(const NodeHybrid::Coordinates&)>;
  using PlanT = std::vector<geometry_msgs::PoseStamped>;
  using PlanHybridT = std::optional<NodeHybrid::CoordinatesVector>;
  using EdgeT = std::pair<NodeHybrid::Coordinates, NodeHybrid ::Coordinates>;
  using TreeMsg = std::vector<EdgeT>;

  /**
   * @brief Empty constructor for planner
   */
  RRTPlannerHybrid();

  /**
   * @brief Constructor for planner
   * @param name Name of the planner
   * @param costmap_ros Costmap 2D ROS pointer
   */
  RRTPlannerHybrid(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destructor for planner
   */
  ~RRTPlannerHybrid();

  /**
   * @brief Initializes planner
   * @param name Name of the planner
   * @param costmap_ros Costmap 2D ROS pointer
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Main planning method
   * @param start Start pose for planning
   * @param goal Goal pose for planning
   * @param plan Generated plan
   * @return True if planning was successful, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, PlanT& plan);

 protected:
  /**
   * @brief Loads planning parameters from param server
   */
  void LoadParams();

  /**
   * @brief Creates path using RRT* algorithm
   * @param start_mx Start X position in map frame
   * @param start_my Start Y position in map frame
   * @param start_orientation_bin Start pose orientation bin index
   * @param goal_mx Goal X position in map frame
   * @param goal_my Goal Y position in map frame
   * @param goal_orientation_bin Goal pose orientation bin index
   * @return Hybrid plan if planning was successful, nullopt otherwise
   */
  PlanHybridT CreatePath(const unsigned int& start_mx,
                         const unsigned int& start_my,
                         const unsigned int& start_orientation_bin,
                         const unsigned int& goal_mx,
                         const unsigned int& goal_my,
                         const unsigned int& goal_orientation_bin);

  /**
   * @brief Process path received from planner
   * @return PlanT Final plan
   */
  PlanT ProcessPath(NodeHybrid::CoordinatesVector& path);

  /**
   * @brief Publishes plan and search tree visualization
   */
  inline void PublishVisualization() {
    visualization_handler_->PublishVisualization();
  }

  /**
   * @brief Clears plan and search tree visualization
   */
  inline void ClearVisualization() {
    visualization_handler_->ClearVisualization();
  }

  /**
   * @brief Updates plan and search tree visualization
   * @param plan
   * @param start_tree
   * @param goal_tree
   */
  void UpdateVisualization(const PlanT& plan, const TreeMsg& start_tree,
                           const TreeMsg& goal_tree);

  bool initialized_{false};
  // Costmap 2D pointer
  costmap_2d::Costmap2D* costmap_{nullptr};
  // Private Node handle
  ros::NodeHandle nh_;
  // RRT* planner pointer
  std::unique_ptr<RRTStar<NodeHybrid>> rrt_star_;
  // Visualization handler
  std::shared_ptr<Visualization<NodeHybrid>> visualization_handler_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
  // Motion model
  MotionModel motion_model_;
  // Search info
  SearchInfo search_info_;
  // Angle bin size
  unsigned int angle_bin_size_{0};
  // Angle bin
  double angle_bin_{0.0};
};

}  // namespace rrt_planner

#endif