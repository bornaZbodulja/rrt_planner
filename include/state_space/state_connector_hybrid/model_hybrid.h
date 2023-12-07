/**
 * @file model_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Model hybrid definition
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_HYBRID__MODEL_HYBRID_H_
#define STATE_SPACE__STATE_CONNECTOR_HYBRID__MODEL_HYBRID_H_

#include "state_space/state_connector_hybrid/motion_model_hybrid.h"

namespace state_space::state_connector_hybrid {
struct HybridModel {
  HybridModel() = default;
  HybridModel(double min_turning_radius_in,
              HybridMotionModel hybrid_motion_model_in)
      : min_turning_radius(min_turning_radius_in),
        hybrid_motion_model(hybrid_motion_model_in) {}

  double min_turning_radius{0.0};
  HybridMotionModel hybrid_motion_model{HybridMotionModel::UNKNOWN};
};

}  // namespace state_space::state_connector_hybrid

#endif
