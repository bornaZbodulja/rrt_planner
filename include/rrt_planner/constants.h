/**
 * @file constants.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-06-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__CONSTANTS_H_
#define RRT_PLANNER__CONSTANTS_H_

#include <string>

namespace rrt_planner {

enum class MotionModel { UNKNOWN = 0, DUBINS = 1, REEDS_SHEPP = 2 };

inline std::string ToString(const MotionModel& motion_model) {
  switch (motion_model) {
    case MotionModel::DUBINS:
      return "Dubins";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline MotionModel FromString(const std::string& motion_model) {
  if (motion_model == "DUBINS") {
    return MotionModel::DUBINS;
  } else if (motion_model == "REEDS-SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}

}  // namespace rrt_planner

#endif
