/**
 * @file motion_model.h
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

enum class MotionModel { UNKNOWN = 0, TDM = 1, DUBINS = 2, REEDS_SHEPP = 3 };

inline std::string MotionModelToString(const MotionModel& motion_model) {
  switch (motion_model) {
    case MotionModel::TDM:
      return "2D";
    case MotionModel::DUBINS:
      return "Dubins";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline MotionModel MotionModelFromString(const std::string& motion_model) {
  if (motion_model == "2D") {
    return MotionModel::TDM;
  } else if (motion_model == "DUBINS") {
    return MotionModel::DUBINS;
  } else if (motion_model == "REEDS-SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}

}  // namespace rrt_planner

#endif
