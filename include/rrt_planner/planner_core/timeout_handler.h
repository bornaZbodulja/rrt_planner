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

  explicit TimeoutHandler(double timeout) : timeout_(timeout) {}

  inline void setStartTime() { start_time_ = getCurrentTime(); }

  inline bool timeoutReached() const {
    return std::chrono::duration_cast<std::chrono::seconds>(getCurrentTime() -
                                                            start_time_)
               .count() > timeout_;
  }

  inline double getElapsedTime() const {
    return std::chrono::duration_cast<std::chrono::seconds>(getCurrentTime() -
                                                            start_time_)
        .count();
  }

 private:
  inline TimeT getCurrentTime() const {
    return std::chrono::system_clock::now();
  }

  double timeout_;
  TimeT start_time_;
};
}  // namespace rrt_planner::planner_core
