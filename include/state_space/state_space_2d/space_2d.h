/**
 * @file space_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Space 2D implementation
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_2D__SPACE_2D_H_
#define STATE_SPACE__STATE_SPACE_2D__SPACE_2D_H_

#include <vector>

#include "state_space/state_space/space.h"

namespace state_space::state_space_2d {
/**
 * @brief Representation of state space 2D dimensions
 */
class Space2D : public state_space::Space {
 public:
  Space2D() = default;
  explicit Space2D(double size_x_in, double size_y_in)
      : size_x(size_x_in), size_y(size_y_in) {}

  ~Space2D() override = default;

  /**
   * @brief Gets number of dimensions in state space
   * @return unsigned int
   */
  unsigned int getDimensions() const override { return 2; }

  /**
   * @brief Gets bounds for each dimension of state space
   * @return std::vector<double>
   */
  std::vector<double> getBounds() const override { return {size_x, size_y}; }

 private:
  double size_x{0}, size_y{0};
};

}  // namespace state_space::state_space_2d

#endif
