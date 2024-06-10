/**
 * @file state_space_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State space hybrid definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_HYBRID__STATE_SPACE_HYBRID_H_
#define STATE_SPACE__STATE_SPACE_HYBRID__STATE_SPACE_HYBRID_H_

#include <nav_utils/collision_checker.h>

#include <cmath>

#include "state_space/state_space/state_space.h"
#include "state_space/state_space_hybrid/space_hybrid.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace state_space::state_space_hybrid {
class StateSpaceHybrid : public state_space::StateSpace<StateHybrid> {
 public:
  explicit StateSpaceHybrid(SpaceHybrid&& space_in)
      : space_(std::move(space_in)) {}
  explicit StateSpaceHybrid(double size_x_in, double size_y_in, double dim_3_in)
      : space_{size_x_in, size_y_in, dim_3_in} {}

  void normalizeState(StateHybrid& state) const override {
    static const double size_x = space_.getBounds().at(0);
    static const double size_y = space_.getBounds().at(1);
    static const double dim_3 = space_.getBounds().at(2);
    state.x = std::fmod(state.x, size_x);
    state.y = std::fmod(state.y, size_y);
    state.theta = std::fmod(state.theta, dim_3);
  }

  StateHybrid getStatesDifference(
      const StateHybrid& start_state,
      const StateHybrid& goal_state) const override {
    return (goal_state - start_state);
  }

  double getStateCost(const StateHybrid& state,
                      const nav_utils::CollisionChecker* const
                          collision_checker) const override {
    return collision_checker->getCost(static_cast<unsigned int>(state.x),
                                      static_cast<unsigned int>(state.y));
  }

  std::vector<double> getBounds() const override { return space_.getBounds(); }

  /**
   * @brief Gets angular bin from raw orientation
   * @param theta Raw orientation
   * @return double Angular bin
   */
  double getAngularBin(double theta) const {
    static const double angle_bin = 2 * M_PI / space_.getBounds().at(2);
    return theta / angle_bin;
  }

  /**
   * @brief Gets raw orientation from bin
   * @param bin_idx Bin index
   * @return double
   */
  double getAngleFromBin(double bin_idx) const {
    static const double angle_bin = 2 * M_PI / space_.getBounds().at(2);
    return bin_idx * angle_bin;
  }

 private:
  SpaceHybrid space_;
};
}  // namespace state_space::state_space_hybrid

#endif  // STATE_SPACE__STATE_SPACE_HYBRID__STATE_SPACE_HYBRID_H_
