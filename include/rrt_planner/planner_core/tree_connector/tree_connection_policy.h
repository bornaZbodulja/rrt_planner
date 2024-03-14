/**
 * @file tree_connection_policy.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines possible tree connection policies
 * @version 0.1
 * @date 2024-02-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTION_POLICY_H_
#define RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTION_POLICY_H_

#include <string>

namespace rrt_planner::planner_core::tree_connector {
/**
 * @brief Representation of possible tree connection policies
 */
enum class TreeConnectionPolicy { UNKNOWN = 0, NEAREST_NEIGHBOR = 1 };

inline const char* treeConnectionPolicyToString(
    TreeConnectionPolicy tree_connection_policy) {
  switch (tree_connection_policy) {
    case TreeConnectionPolicy::NEAREST_NEIGHBOR:
      return "Nearest neighbor";
    default:
      return "Unknown";
  }
}

inline TreeConnectionPolicy treeConnectionPolicyFromString(
    const std::string& tree_connection_policy) {
  if (tree_connection_policy == "NEAREST_NEIGHBOR") {
    return TreeConnectionPolicy::NEAREST_NEIGHBOR;
  } else {
    return TreeConnectionPolicy::UNKNOWN;
  }
}
}  // namespace rrt_planner::planner_core::tree_connector

#endif  // RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTION_POLICY_H_
