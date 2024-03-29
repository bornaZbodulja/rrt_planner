/**
 * @file space_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Space hybrid definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_SPACE_HYBRID__SPACE_HYBRID_H_
#define STATE_SPACE__STATE_SPACE_HYBRID__SPACE_HYBRID_H_

#include <cmath>
#include <vector>

#include "state_space/state_space/space.h"

namespace state_space::state_space_hybrid {
/**
 * @brief Representation of state space hybrid dimensions
 */
class SpaceHybrid : public state_space::Space {
 public:
  SpaceHybrid() = default;
  explicit SpaceHybrid(double size_x_in, double size_y_in, double dim_3_in)
      : size_x(size_x_in), size_y(size_y_in), dim_3(dim_3_in) {
    angle_bin = 2 * M_PI / dim_3;
  }

  ~SpaceHybrid() override = default;

  /**
   * @brief Gets number of dimensions in state space
   * @return unsigned int
   */
  unsigned int getDimensions() const override { return 3; }

  /**
   * @brief Gets bounds for each dimension of state space
   * @return std::vector<double>
   */
  std::vector<double> getBounds() const override {
    return {size_x, size_y, dim_3};
  }

  double size_x{0}, size_y{0}, dim_3{0};
  double angle_bin{0.0};
};

}  // namespace state_space::state_space_hybrid

#endif
