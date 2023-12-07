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

#include <cmath>

#include "state_space/state_space/state_space.h"
#include "state_space/state_space_hybrid/space_hybrid.h"
#include "state_space/state_space_hybrid/state_hybrid.h"

namespace state_space::state_space_hybrid {
class StateSpaceHybrid : public state_space::StateSpace<StateHybrid> {
 public:
  StateSpaceHybrid(const SpaceHybrid& space_in) : space_(space_in) {}
  StateSpaceHybrid(unsigned int size_x_in, unsigned int size_y_in,
                   unsigned int dim_3_in)
      : space_{size_x_in, size_y_in, dim_3_in} {}

  inline unsigned int getIndex(const StateHybrid& state) const override {
    return state.theta + state.x * space_.dim_3 +
           state.y * space_.size_x * space_.dim_3;
  }

  inline StateHybrid getState(unsigned int index) const override {
    return StateHybrid((index / space_.dim_3) % space_.size_x,
                       index / (space_.dim_3 * space_.size_x),
                       index % space_.dim_3);
  }

  inline void normalizeState(StateHybrid& state) const override {
    state.x = std::fmod(state.x, static_cast<double>(space_.size_x));
    state.y = std::fmod(state.y, static_cast<double>(space_.size_y));
    state.theta = std::fmod(state.theta, static_cast<double>(space_.dim_3));
  }

  inline StateHybrid getStateDistance(
      const StateHybrid& start_state,
      const StateHybrid& goal_state) const override {
    return (goal_state - start_state);
  }

  inline unsigned int getStateSpaceSize() const override {
    return space_.size_x * space_.size_y * space_.dim_3;
  }

  /**
   * @brief Gets angular bin index
   * @param theta Raw orientation
   * @return unsigned int Index of bin
   */
  inline unsigned int getClosestAngularBin(double theta) const {
    return static_cast<unsigned int>(std::floor(theta / space_.angle_bin));
  }

  /**
   * @brief Gets raw orientation from bin
   * @param bin_idx Bin index
   * @return double
   * TODO: Make this method use double instead of unsigned int
   */
  inline double getAngleFromBin(unsigned int bin_idx) {
    return bin_idx * space_.angle_bin;
  }

 protected:
  SpaceHybrid space_;
};

}  // namespace state_space::state_space_hybrid

#endif
