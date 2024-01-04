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

#include <chrono>

namespace rrt_planner::planner_core {
/**
 * @brief
 */
class TimeoutHandler {
 public:
  using TimeT = std::chrono::time_point<std::chrono::system_clock>;

  /**
   * @brief
   * @param timeout Timeout in seconds
   */
  explicit TimeoutHandler(double timeout) : timeout_(timeout * 1000.0) {}

  void setStartTime() { start_time_ = getCurrentTime(); }

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
  TimeT start_time_;
};
}  // namespace rrt_planner::planner_core
