/**
 * @file search_policy_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__SEARCH_POLICY_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__SEARCH_POLICY_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "rrt_planner/planner_core/search_policy.h"

namespace rrt_planner::param_loader {

inline rrt_planner::planner_core::SearchPolicy loadSearchPolicy(
    ros::NodeHandle* nh) {
  std::string search_policy_str;
  loadParam<std::string>(nh, "search_policy", search_policy_str);
  auto search_policy =
      rrt_planner::planner_core::searchPolicyFromString(search_policy_str);

  if (search_policy == rrt_planner::planner_core::SearchPolicy::UNKNOWN) {
    ROS_ERROR("Unknown search policy for planner: %s. Terminating the program!",
              search_policy_str.c_str());
    exit(0);
  }

  return search_policy;
}

}  // namespace rrt_planner::param_loader

#endif
