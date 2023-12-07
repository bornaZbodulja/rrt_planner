/**
 * @file visualization_utilities.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Utility functions for visualization
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_UTILITIES_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_UTILITIES_H_

#include <std_msgs/ColorRGBA.h>

namespace rrt_planner::visualization {
inline std_msgs::ColorRGBA colorRed() {
  std_msgs::ColorRGBA red;
  red.a = 1.0;
  red.r = 1.0;
  return red;
}

inline std_msgs::ColorRGBA colorGreen() {
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.g = 1.0;
  return green;
}

}  // namespace rrt_planner::visualization

#endif
