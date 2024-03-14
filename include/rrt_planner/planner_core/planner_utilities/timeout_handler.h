/**
 * @file planning_time_handler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planning timeout handler
 * @version 0.1
 * @date 2023-10-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_UTILITIES__TIMEOUT_HANDLER_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_UTILITIES__TIMEOUT_HANDLER_H_

#include <chrono>

namespace rrt_planner::planner_core::planner_utilities {
/**
 * @brief Class for monitoring planning time
 */
class TimeoutHandler {
 public:
  using TimeT = std::chrono::time_point<std::chrono::system_clock>;

  /**
   * @brief Class constructor
   * @param timeout Timeout in seconds
   */
  explicit TimeoutHandler(double timeout) : timeout_(timeout * 1000.0) {}

  void setStartTime() { start_time_ = getCurrentTime(); }

  /**
   * @brief Checks if timeout has been reached
   * @return True if timeout reached, false otherwise
   */
  bool timeoutReached() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               getCurrentTime() - start_time_)
               .count() > timeout_;
  }

  double getElapsedTime() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               getCurrentTime() - start_time_)
               .count() /
           1000.0;
  }

 private:
  TimeT getCurrentTime() const { return std::chrono::system_clock::now(); }

  // Timeout in milliseconds
  double timeout_;
  // Start time
  TimeT start_time_{std::chrono::system_clock::now()};
};
}  // namespace rrt_planner::planner_core::planner_utilities

#endif
