/**
 * @file state_sampler.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State sampler interface
 * @version 0.1
 * @date 2023-10-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <nav_utils/nav_utils.h>

#include "state_space/state_sampler/state_sampler_params.h"
#include "state_space/state_space/state_space.h"

#ifndef STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_H_
#define STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_H_

namespace state_space::state_sampler {
template <typename StateT>
class StateSampler {
 public:
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = StateSpaceT*;

  explicit StateSampler(StateSamplerParams&& params)
      : params_(std::move(params)) {}

  virtual ~StateSampler() = default;

  /**
   * @brief Generates new index for search tree expansion
   * @param target_index Index of target node of the search tree
   * @param state_space State space pointer
   * @param collision_checker Collision checker pointer
   * @return unsigned int
   */
  virtual unsigned int generateTreeExpansionIndex(
      unsigned int target_index, StateSpacePtr state_space,
      const CollisionCheckerPtr& collision_checker) = 0;

 protected:
  StateSamplerParams params_;
};
}  // namespace state_space::state_sampler

#endif
