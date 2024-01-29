/**
 * @file search_policy.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines possible search policies for planner
 * @version 0.1
 * @date 2023-09-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__SEARCH_POLICY_H_
#define RRT_PLANNER__PLANNER_CORE__SEARCH_POLICY_H_

#include <string>

namespace rrt_planner::planner_core {
/**
 * @brief Representation of possible search policies for planner
 */
enum class SearchPolicy {
  UNKNOWN = 0,
  RRT = 1,
  BIDIRECTIONAL_RRT = 2,
  RRT_STAR = 3,
  BIDIRECTIONAL_RRT_STAR = 4
};

inline std::string searchPolicyToString(SearchPolicy search_policy) {
  switch (search_policy) {
    case SearchPolicy::RRT:
      return "RRT";
    case SearchPolicy::BIDIRECTIONAL_RRT:
      return "Bidirectional RRT";
    case SearchPolicy::RRT_STAR:
      return "RRT*";
    case SearchPolicy::BIDIRECTIONAL_RRT_STAR:
      return "Bidirectional RRT*";
    default:
      return "Unknown";
  }
}

inline SearchPolicy searchPolicyFromString(const std::string& search_policy) {
  if (search_policy == "RRT") {
    return SearchPolicy::RRT;
  } else if (search_policy == "BIDIRECTIONAL_RRT") {
    return SearchPolicy::BIDIRECTIONAL_RRT;
  } else if (search_policy == "RRT_STAR") {
    return SearchPolicy::RRT_STAR;
  } else if (search_policy == "BIDIRECTIONAL_RRT_STAR") {
    return SearchPolicy::BIDIRECTIONAL_RRT_STAR;
  } else {
    return SearchPolicy::UNKNOWN;
  }
}
}  // namespace rrt_planner::planner_core

#endif
