/**
 * @file motion_model_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Hybrid motion models
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STATE_SPACE__STATE_CONNECTOR_HYBRID__MOTION_MODEL_HYBRID_H_
#define STATE_SPACE__STATE_CONNECTOR_HYBRID__MOTION_MODEL_HYBRID_H_

#include <string>

namespace state_space::state_connector_hybrid {
/**
 * @brief Representation of hybrid motion models
 */
enum class HybridMotionModel { UNKNOWN = 0, DUBINS = 1, REEDS_SHEPP = 2 };

inline std::string hybridMotionModelToString(HybridMotionModel motion_model) {
  switch (motion_model) {
    case HybridMotionModel::DUBINS:
      return "Dubins";
    case HybridMotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline HybridMotionModel hybridMotionModelFromString(std::string motion_model) {
  if (motion_model == "DUBINS") {
    return HybridMotionModel::DUBINS;
  } else if (motion_model == "REEDS-SHEPP") {
    return HybridMotionModel::REEDS_SHEPP;
  } else {
    return HybridMotionModel::UNKNOWN;
  }
}

}  // namespace state_space::state_connector_hybrid

#endif
