/**
 * @file param_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Utility for loading params from ROS param server
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__PARAM_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__PARAM_LOADER_H_

#include <ros/console.h>
#include <ros/node_handle.h>

#include <string>

namespace rrt_planner::param_loader {

/**
 * @brief
 * @tparam T
 * @param nh
 * @param name
 * @param value
 */
template <typename T>
inline void loadParam(ros::NodeHandle* nh, std::string name, T& value) {
  if (!nh->getParam(name, value)) {
    ROS_ERROR("Parameter %s doesn't exists, terminating the program!",
              (nh->getNamespace() + "/" + name).c_str());
    exit(0);
  }
}

/**
 * @brief
 * @tparam
 * @param nh
 * @param name
 * @param value
 */
template <>
inline void loadParam<unsigned char>(ros::NodeHandle* nh, std::string name,
                                     unsigned char& value) {
  int int_value;
  loadParam<int>(nh, name, int_value);
  value = static_cast<unsigned char>(int_value);
}
}  // namespace rrt_planner::param_loader

#endif // RRT_PLANNER__PARAM_LOADER__PARAM_LOADER_H_
