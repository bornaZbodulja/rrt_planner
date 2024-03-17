/**
 * @file map_info.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Holder for basic map info
 * @version 0.1
 * @date 2024-03-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_PLUGIN__MAP_INFO_H_
#define RRT_PLANNER__PLANNER_PLUGIN__MAP_INFO_H_

namespace rrt_planner::planner_plugin {
/**
 * @brief Representation of basic map info
 */
struct MapInfo {
  MapInfo() = default;
  MapInfo(double origin_x_in, double origin_y_in, double size_x_in,
          double size_y_in, double resolution_in)
      : origin(origin_x_in, origin_y_in),
        size(size_x_in, size_y_in),
        resolution(resolution_in) {}

  /**
   * @brief Representation of map origin
   */
  struct Origin {
    Origin() = default;
    Origin(double x_in, double y_in) : x(x_in), y(y_in) {}

    double x{0.0};
    double y{0.0};
  } origin{};

  /**
   * @brief Representation of map size
   */
  struct Size {
    Size() = default;
    Size(double x_in, double y_in) : x(x_in), y(y_in) {}

    double x{0.0};
    double y{0.0};
  } size{};

  double resolution{0.0};
};
}  // namespace rrt_planner::planner_plugin

#endif  // RRT_PLANNER__PLANNER_PLUGIN__MAP_INFO_H_
