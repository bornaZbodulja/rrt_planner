/**
 * @file analytic_motion_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Analytic state connector for hybrid motion
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_HYBRID__ANALYTIC_MOTION_HYBRID_H_
#define STATE_SPACE__STATE_CONNECTOR_HYBRID__ANALYTIC_MOTION_HYBRID_H_

#include <nav_utils/nav_utils.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_connector_hybrid/model_hybrid.h"
#include "state_space/state_connector_hybrid/motion_model_hybrid.h"
#include "state_space/state_space_hybrid/state_hybrid.h"
#include "state_space/state_space_hybrid/state_space_hybrid.h"

namespace state_space::state_connector_hybrid {
/**
 * @brief
 */
class AnalyticMotionHybrid {
 public:
  using StateT = state_space::state_space_hybrid::StateHybrid;
  using StateSpaceT = state_space::state_space_hybrid::StateSpaceHybrid;
  using StateSpacePtr = StateSpaceT*;
  using StateVector = std::vector<StateT>;
  using ExpansionResultT = std::optional<StateT>;
  using ConnectionParamsT = state_space::state_connector::StateConnectorParams;

  explicit AnalyticMotionHybrid(const HybridModel& hybrid_model);
  ~AnalyticMotionHybrid() = default;

  /**
   * @brief Tries analytically expanding state towards target state
   * @param start
   * @param target
   * @param state_space
   * @param connection_params
   * @param collision_checker
   * @return ExpansionResultT
   */
  ExpansionResultT tryAnalyticExpand(
      const StateT& start, const StateT& target,
      const StateSpacePtr& state_space,
      const ConnectionParamsT& connection_params,
      const CollisionCheckerPtr& collision_checker) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param state_space
   * @param connection_params
   * @param collision_checker
   * @return true
   * @return false
   */
  bool tryAnalyticConnect(const StateT& start, const StateT& goal,
                          const StateSpacePtr& state_space,
                          const ConnectionParamsT& connection_params,
                          const CollisionCheckerPtr& collision_checker) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param state_space
   * @return StateVector
   */
  StateVector getAnalyticPath(const StateT& start, const StateT& goal,
                              const StateSpacePtr& state_space) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param state_space
   * @return double
   */
  double getAnalyticPathLength(const StateT& start, const StateT& goal,
                               const StateSpacePtr& state_space) const;

 private:
  bool isMotionModelValid() const {
    return (hybrid_model_.hybrid_motion_model == HybridMotionModel::DUBINS) ||
           (hybrid_model_.hybrid_motion_model ==
            HybridMotionModel::REEDS_SHEPP);
  }

  double getAnalyticStateDistance(ompl::base::ScopedState<>& start,
                                  ompl::base::ScopedState<>& goal) const {
    return ompl_state_space_->distance(start(), goal());
  }

  /**
   * @brief Computes number of states for path interpolation
   * @param start
   * @param goal
   * @return int
   */
  int computeConnectionStatesNum(ompl::base::ScopedState<>& start,
                                 ompl::base::ScopedState<>& goal) const;

  /**
   * @brief Populates OMPL state from hybrid state
   * @param ompl_state
   * @param state
   * @param state_space
   */
  void stateToOMPLState(ompl::base::ScopedState<>& ompl_state,
                        const StateT& state,
                        const StateSpacePtr& state_space) const;

  // OMPL state space pointer
  ompl::base::StateSpacePtr ompl_state_space_;
  // Hybrid model for analytic motion
  HybridModel hybrid_model_;
};

}  // namespace state_space::state_connector_hybrid

#endif
