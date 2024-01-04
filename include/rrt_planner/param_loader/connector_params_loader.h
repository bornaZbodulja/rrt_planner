/**
 * @file connector_params_loader.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PARAM_LOADER__CONNECTOR_PARAMS_LOADER_H_
#define RRT_PLANNER__PARAM_LOADER__CONNECTOR_PARAMS_LOADER_H_

#include "rrt_planner/param_loader/param_loader.h"
#include "state_space/state_connector/state_connector_params.h"

namespace rrt_planner::param_loader {

/**
 * @brief
 *
 * @param nh
 * @param costmap_resolution
 * @return state_space::state_connector::StateConnectorParams
 */
state_space::state_connector::StateConnectorParams
loadStateConnectorParams(ros::NodeHandle* nh, double costmap_resolution) {
  state_space::state_connector::StateConnectorParams connector_params;
  loadParam<bool>(nh, "allow_unknown", connector_params.allow_unknown);
  loadParam<unsigned char>(nh, "lethal_cost", connector_params.lethal_cost);

  double edge_length;
  loadParam<double>(nh, "edge_length", edge_length);
  // Scaling with costmap resolution
  connector_params.max_extension_states =
      static_cast<int>(edge_length / costmap_resolution);

  return connector_params;
}

}  // namespace rrt_planner::param_loader

#endif
