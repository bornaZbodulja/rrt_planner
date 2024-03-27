/**
 * @file planner.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planner interface
 * @version 0.1
 * @date 2024-02-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_

#include <optional>
#include <vector>

namespace rrt_planner::planner_core::planner {
/**
 * @brief Planner interface
 * @tparam StateT
 */
template <typename StateT>
class Planner {
 public:
  virtual ~Planner() = default;

  /**
   * @brief Initializes internal search structures
   */
  virtual void initializeSearch() = 0;

  /**
   * @brief Sets start node for search
   * @param start_state Start state
   */
  virtual void setStart(const StateT& start_state) = 0;

  /**
   * @brief Sets goal node for search
   * @param goal_state Goal state
   */
  virtual void setGoal(const StateT& goal_state) = 0;

  /**
   * @brief Tries to plan a path from setted start to goal
   * @return std::optional<std::vector<StateT>>
   */
  virtual std::optional<std::vector<StateT>> createPath() = 0;

  /**
   * @brief Returns search trees for visualization
   * @return std::vector<std::vector<StateT>>
   */
  virtual std::vector<std::vector<std::vector<StateT>>> getTrees() const = 0;

 protected:
  Planner() = default;
};
}  // namespace rrt_planner::planner_core::planner

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_
