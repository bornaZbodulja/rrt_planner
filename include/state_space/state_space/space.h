/**
 * @file space.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Space interface
 * @version 0.1
 * @date 2024-03-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef STATE_SPACE__STATE_SPACE__SPACE_H_
#define STATE_SPACE__STATE_SPACE__SPACE_H_

#include <vector>

namespace state_space {
/**
 * @brief Space interface
 */
class Space {
 public:
  virtual ~Space() = default;

  /**
   * @brief Gets number of dimensions in state space
   * @return unsigned int
   */
  virtual unsigned int getDimensions() const = 0;

  /**
   * @brief Gets bounds for each dimension of state space
   * @return std::vector<double>
   */
  virtual std::vector<double> getBounds() const = 0;

 protected:
  Space() = default;
};
}  // namespace state_space

#endif  //  STATE_SPACE__STATE_SPACE__SPACE_H_
