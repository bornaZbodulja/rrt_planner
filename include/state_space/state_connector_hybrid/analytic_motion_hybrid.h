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

#include <nav_utils/collision_checker.h>
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
  using StateHybrid = state_space::state_space_hybrid::StateHybrid;

  explicit AnalyticMotionHybrid(
      HybridModel&& hybrid_model,
      state_space::state_connector::StateConnectorParams&& connector_params,
      const std::shared_ptr<state_space::state_space_hybrid::StateSpaceHybrid>&
          state_space,
      const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker);

  ~AnalyticMotionHybrid() = default;

  /**
   * @brief Tries analytically expanding state towards target state
   * @param start
   * @param target
   * @param state_space
   * @param connection_params
   * @param collision_checker
   * @return std::optional<StateHybrid>
   */
  std::optional<StateHybrid> tryAnalyticExpand(const StateHybrid& start,
                                               const StateHybrid& target) const;

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
  bool tryAnalyticConnect(const StateHybrid& start,
                          const StateHybrid& goal) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param state_space
   * @return std::vector<StateHybrid>
   */
  std::vector<StateHybrid> getAnalyticPath(const StateHybrid& start,
                                           const StateHybrid& goal) const;

  /**
   * @brief
   * @param start
   * @param goal
   * @param state_space
   * @return double
   */
  double getAnalyticPathLength(const StateHybrid& start,
                               const StateHybrid& goal) const;

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
   */
  void StateHybridToOMPLState(ompl::base::ScopedState<>& ompl_state,
                              const StateHybrid& state) const;

  /**
   * @brief
   * @param x X coordinate
   * @param y Y coordinate
   * @param yaw Orientation
   * @return True if pose is in collision, false otherwise
   */
  bool poseInCollision(double x, double y, double yaw) const;

  // Hybrid model for analytic motion
  HybridModel hybrid_model_;
  // Connector parameters
  state_space::state_connector::StateConnectorParams connector_params_;
  // State space pointer
  std::shared_ptr<state_space::state_space_hybrid::StateSpaceHybrid>
      state_space_;
  // Collision checker pointer
  std::shared_ptr<nav_utils::CollisionChecker> collision_checker_;
  // OMPL state space pointer
  ompl::base::StateSpacePtr ompl_state_space_;
};

}  // namespace state_space::state_connector_hybrid

#endif
