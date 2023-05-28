/**
 * @file visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <functional>
#include <utility>
#include <vector>

#include "rrt_planner/node_2d.h"

namespace rrt_planner {

/**
 * @brief Tree id enum
 */
enum TreeId { START_TREE = 0, GOAL_TREE = 1 };

/**
 * @brief Visualization handler for RRT* planner
 * @tparam NodeT
 */
template <typename NodeT>
class Visualization {
 public:
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::pair<Coordinates, Coordinates> Edge;
  typedef std::vector<Edge> TreeMsg;
  typedef std::function<geometry_msgs::Pose(const Coordinates&)>
      WorldCoordinatesGetter;

  /**
   * @brief Constructor for visualization handler
   * @param nh Node handle
   * @param world_coordinates_getter World coordinates getter
   */
  Visualization(ros::NodeHandle* nh,
                WorldCoordinatesGetter& world_coordinates_getter);

  /**
   * @brief Destructor for visualization handler
   */
  ~Visualization();

  /**
   * @brief Publishes path and search tree visualization
   */
  void PublishVisualization() {
    PublishPath();
    PublishSearchTree();
  }

  /**
   * @brief Clears path and search tree visualization
   */
  void ClearVisualization();

  /**
   * @brief Sets plan for visualization
   * @param plan
   */
  void SetPathVisualization(
      const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Sets search tree for visualization
   * @param start_tree Start search tree
   * @param goal_tree Goal search tree
   */
  void SetSearchTreeVisualization(const TreeMsg& start_tree,
                                  const TreeMsg& goal_tree);

 private:
  /**
   * @brief Publishes planned path visualization
   */
  inline void PublishPath() { plan_pub_.publish(path_); }

  /**
   * @brief Publishes search tree visualization
   */
  inline void PublishSearchTree() { tree_pub_.publish(search_tree_); }

  /**
   * @brief Clears planned path visualization
   */
  void ClearPath();

  /**
   * @brief Clears search tree visualization
   */
  void ClearSearchTree();

  /**
   * @brief Adds tree to visualization
   * @param tree
   * @param id Id of the tree
   */
  void AddTree(const TreeMsg& tree, const TreeId& id);

  /**
   * @brief Returns color of the tree based on id
   * @param id tree id
   * @return std_msgs::ColorRGBA
   */
  std_msgs::ColorRGBA TreeColorMapper(const TreeId& id) {
    switch (id) {
      case TreeId::START_TREE:
        return ColorRed();
      case TreeId::GOAL_TREE:
        return ColorGreen();
      default:
        return ColorEmpty();
    }
  }

  /**
   * @brief RGB format of red color
   * @return std_msgs::ColorRGBA
   */
  std_msgs::ColorRGBA ColorRed() {
    std_msgs::ColorRGBA red;
    red.a = 1.0;
    red.r = 1.0;
    return red;
  }

  /**
   * @brief RGB format of green color
   * @return std_msgs::ColorRGBA
   */
  std_msgs::ColorRGBA ColorGreen() {
    std_msgs::ColorRGBA green;
    green.a = 1.0;
    green.g = 1.0;
    return green;
  }

  /**
   * @brief Empty RGB format
   * @return std_msgs::ColorRGBA
   */
  std_msgs::ColorRGBA ColorEmpty() {
    std_msgs::ColorRGBA empty;
    return empty;
  }

  // Plan publisher
  ros::Publisher plan_pub_;
  // Search tree publisher
  ros::Publisher tree_pub_;
  // Visualization of planned path
  nav_msgs::Path path_;
  // Visualization of search tree
  visualization_msgs::MarkerArray search_tree_;
  // World coordinates getter
  WorldCoordinatesGetter world_coords_getter_;
};

}  // namespace rrt_planner

#endif
