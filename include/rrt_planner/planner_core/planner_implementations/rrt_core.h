/**
 * @file rrt_core.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT planner interface
 * @version 0.1
 * @date 2024-03-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_CORE_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_CORE_H_

#include <ros/console.h>

#include <memory>

#include "rrt_planner/planner_core/planner/planner.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_implementations/search_params.h"
#include "rrt_planner/planner_core/planner_implementations/search_policy.h"
#include "rrt_planner/planner_core/planner_utilities/timeout_handler.h"

namespace rrt_planner::planner_core::planner_implementations {
/**
 * @brief RRT planner core interface
 * @tparam State
 */
template <typename StateT>
class RRTCore : public rrt_planner::planner_core::planner::Planner<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

  ~RRTCore() override = default;

  void initializeSearch() override { iterations_counter_ = 0; }

 protected:
  RRTCore(rrt_planner::planner_core::planner_implementations::SearchPolicy
              search_policy,
          rrt_planner::planner_core::planner_implementations::SearchParams&&
              search_params)
      : search_policy_(search_policy),
        timeout_handler_(
            std::make_unique<
                rrt_planner::planner_core::planner_utilities::TimeoutHandler>(
                search_params_.max_planning_time)),
        search_params_(std::move(search_params)) {}

  void setPlanningStartTime() { timeout_handler_->setStartTime(); }

  bool planningExpired() const {
    return isPlanningTimeoutReached() || reachedMaximumIterations();
  }

  void logSuccessfulPathCreation() const {
    ROS_INFO(
        "%s planner found path, used iterations %d/%d, planning time: %.3f "
        "seconds.",
        rrt_planner::planner_core::planner_implementations::
            searchPolicyToString(search_policy_),
        iterations_counter_, search_params_.max_expansion_iterations,
        getElapsedTime());
  }

  void logUnsuccessfulPathCreation() const {
    ROS_WARN(
        "%s planner unable to find path, used iterations %d/%d, planning time: "
        "%.3f seconds.",
        rrt_planner::planner_core::planner_implementations::
            searchPolicyToString(search_policy_),
        iterations_counter_, search_params_.max_expansion_iterations,
        getElapsedTime());
  }

 private:
  double getElapsedTime() const { return timeout_handler_->getElapsedTime(); }

  bool isPlanningTimeoutReached() const {
    return timeout_handler_->timeoutReached();
  }

  bool reachedMaximumIterations() const {
    return iterations_counter_ >= search_params_.max_expansion_iterations;
  }

  // Search policy
  rrt_planner::planner_core::planner_implementations::SearchPolicy
      search_policy_;
  // Planning timeout handler pointer
  std::unique_ptr<rrt_planner::planner_core::planner_utilities::TimeoutHandler>
      timeout_handler_;
  // Planning expansions iterations counter
  int iterations_counter_{0};

 protected:
  // Search parameters
  rrt_planner::planner_core::planner_implementations::SearchParams
      search_params_;
};
}  // namespace rrt_planner::planner_core::planner_implementations

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_IMPLEMENTATIONS__RRT_CORE_H_
