/**
 * @file utils.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__UTILS_H_
#define RRT_PLANNER__UTILS_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/utils.h>

#include "nav_utils/nav_utils.h"

namespace rrt_planner {

/**
 * @brief Creates world pose from map coordinates
 * @param mx X coordinate in map frame
 * @param my Y coordinate in map frame
 * @param collision_checker Collision checker pointer
 * @return geometry_msgs::Pose
 */
geometry_msgs::Pose GetWorldCoordinates(
    const unsigned int& mx, const unsigned int& my,
    const CollisionCheckerPtr& collision_checker) {
  geometry_msgs::Pose pose;
  collision_checker->MapToWorld(mx, my, pose.position.x, pose.position.y);
  return pose;
}

geometry_msgs::Quaternion GetWorldOrientation(const double& yaw) {
  return tf2::toMsg(tf2::Quaternion{tf2::Vector3(0, 0, 1), yaw});
}

}  // namespace rrt_planner

#endif
