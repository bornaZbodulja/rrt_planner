/**
 * @file expansion_policy.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Defines possible expansion policies for expander
 * @version 0.1
 * @date 2024-01-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__EXPANDER__EXPANSION_POLICY_H_
#define RRT_PLANNER__PLANNER_CORE__EXPANDER__EXPANSION_POLICY_H_

#include <string>

namespace rrt_planner::planner_core::expander {
/**
 * @brief Representation of possible expansion policies for expander
 */
enum class ExpansionPolicy {
  UNKNOWN = 0,
  NEAREST_NEIGHBOR = 1,
  NEAREST_NEIGHBOR_STAR = 2
};

inline const char* expansionPolicyToString(ExpansionPolicy expansion_policy) {
  switch (expansion_policy) {
    case ExpansionPolicy::NEAREST_NEIGHBOR:
      return "Nearest neighbor";
    case ExpansionPolicy::NEAREST_NEIGHBOR_STAR:
      return "Nearest neighbor*";
    default:
      return "Unknown";
  }
}

inline ExpansionPolicy expansionPolicyFromString(
    const std::string& expansion_policy) {
  if (expansion_policy == "NEAREST_NEIGHBOR") {
    return ExpansionPolicy::NEAREST_NEIGHBOR;
  } else if (expansion_policy == "NEAREST_NEIGHBOR*") {
    return ExpansionPolicy::NEAREST_NEIGHBOR_STAR;
  } else {
    return ExpansionPolicy::UNKNOWN;
  }
}
}  // namespace rrt_planner::planner_core::expander

#endif
