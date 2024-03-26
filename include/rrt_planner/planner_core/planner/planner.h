/**
 * @file planner.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT planner interface
 * @version 0.1
 * @date 2024-02-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_

#include <ros/console.h>

#include <memory>
#include <optional>
#include <vector>

#include "rrt_planner/planner_core/planner/search_params.h"
#include "rrt_planner/planner_core/planner/search_policy.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_graph.h"
#include "rrt_planner/planner_core/planner_utilities/timeout_handler.h"

namespace rrt_planner::planner_core::planner {
/**
 * @brief RRT planner interface
 * @tparam StateT
 */
template <typename StateT>
class Planner {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

  virtual ~Planner() = default;

  /**
   * @brief Initializes search by clearing memory from previous search
   */
  virtual void initializeSearch() {
    graph_->clear();
    iterations_counter_ = 0;
  }

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
  Planner(rrt_planner::planner_core::planner::SearchPolicy search_policy,
          SearchParams&& search_params)
      : search_params_(std::move(search_params)),
        graph_(std::make_unique<rrt_planner::planner_core::planner_entities::
                                    SearchGraph<NodeT>>()),
        timeout_handler_(
            std::make_unique<
                rrt_planner::planner_core::planner_utilities::TimeoutHandler>(
                search_params_.max_planning_time)),
        search_policy_(search_policy) {
    graph_->reserve(search_params_.max_expansion_iterations);
  }

  NodeT* getNodeFromGraph(unsigned int index) { return graph_->getNode(index); }

  void setPlanningStartTime() { timeout_handler_->setStartTime(); }

  double getElapsedTime() const { return timeout_handler_->getElapsedTime(); }

  bool isPlanningTimeoutReached() const {
    return timeout_handler_->timeoutReached();
  }

  bool reachedMaximumNumberOfIterations() {
    return ++iterations_counter_ > search_params_.max_expansion_iterations;
  }

  bool planningExpired() {
    return isPlanningTimeoutReached() || reachedMaximumNumberOfIterations();
  }

  void logSuccessfulPathCreation() const {
    ROS_INFO(
        "%s planner found path, used iterations %d/%d, planning time: %.3f "
        "seconds.",
        rrt_planner::planner_core::planner::searchPolicyToString(
            search_policy_),
        iterations_counter_, search_params_.max_expansion_iterations,
        getElapsedTime());
  }

  void logUnsuccessfulPathCreation() const {
    ROS_WARN(
        "%s planner unable to find path, used iterations %d/%d, planning time: "
        "%.3f seconds.",
        rrt_planner::planner_core::planner::searchPolicyToString(
            search_policy_),
        iterations_counter_, search_params_.max_expansion_iterations,
        getElapsedTime());
  }

  // Search parameters
  SearchParams search_params_;
  // Search graph pointer
  std::unique_ptr<
      rrt_planner::planner_core::planner_entities::SearchGraph<NodeT>>
      graph_;

 private:
  // Planning timeout handler
  std::unique_ptr<rrt_planner::planner_core::planner_utilities::TimeoutHandler>
      timeout_handler_;
  // Search policy
  rrt_planner::planner_core::planner::SearchPolicy search_policy_;
  // Planning expansions iterations counter
  int iterations_counter_{0};
};
}  // namespace rrt_planner::planner_core::planner

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER__PLANNER_H_
