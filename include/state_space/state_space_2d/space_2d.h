/**
 * @file space_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Space 2D definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_2D__SPACE_2D_H_
#define STATE_SPACE__STATE_SPACE_2D__SPACE_2D_H_

namespace state_space::state_space_2d {
/**
 * @brief Representation of state space 2D dimensions
 */
struct Space2D {
  Space2D() = default;
  explicit Space2D(unsigned int size_x_in, unsigned int size_y_in)
      : size_x(size_x_in), size_y(size_y_in) {}

  unsigned int size_x{0}, size_y{0};
};

}  // namespace state_space::state_space_2d

#endif
