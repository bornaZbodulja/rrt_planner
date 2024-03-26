/**
 * @file cost_scorer.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Cost scorer implementation
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__COST_SCORER__COST_SCORER_H_
#define RRT_PLANNER__PLANNER_CORE__COST_SCORER__COST_SCORER_H_

#include <nav_utils/collision_checker.h>

#include <memory>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer_params.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::cost_scorer {
template <typename StateT>
class CostScorer {
 public:
  CostScorer(
      CostScorerParams&& cost_scorer_params,
      const std::shared_ptr<state_space::StateSpace<StateT>>& state_space,
      const std::shared_ptr<
          state_space::state_connector::StateConnector<StateT>>&
          state_connector,
      const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker)
      : params_(std::move(cost_scorer_params)),
        state_space_(state_space),
        state_connector_(state_connector),
        collision_checker_(collision_checker) {}

  virtual ~CostScorer() = default;

  virtual double operator()(const StateT& parent_state,
                            const StateT& child_state) const {
    return computeStateCost(child_state) +
           computeTraversalCost(parent_state, child_state);
  }

 protected:
  virtual double computeStateCost(const StateT& state) const {
    return params_.cost_penalty *
           state_space_->getStateCost(state, collision_checker_.get());
  }

  virtual double computeTraversalCost(const StateT& parent_state,
                                      const StateT& child_state) const {
    return params_.traversal_penalty *
           state_connector_->getStatesDistance(parent_state, child_state);
  }

  // Cost scoring parameters
  CostScorerParams params_;
  // State space pointer
  std::shared_ptr<state_space::StateSpace<StateT>> state_space_;
  // State connector pointer
  std::shared_ptr<state_space::state_connector::StateConnector<StateT>>
      state_connector_;
  // Collision checker pointer
  std::shared_ptr<nav_utils::CollisionChecker> collision_checker_;
};
}  // namespace rrt_planner::planner_core::cost_scorer

#endif
