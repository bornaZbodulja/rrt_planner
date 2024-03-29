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

#ifndef STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_H_
#define STATE_SPACE__STATE_SAMPLER__STATE_SAMPLER_H_

namespace state_space::state_sampler {
template <typename StateT>
class StateSampler {
 public:
  virtual ~StateSampler() = default;

  /**
   * @brief Generates new state for search tree expansion
   * @param target_state State of target node of the search tree
   * @return double
   */
  virtual StateT generateTreeExpansionState(StateT target_state) = 0;

 protected:
  StateSampler() = default;
};
}  // namespace state_space::state_sampler

#endif
