/**
 * @file state_connector_params.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief State connector params definition
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__STATE_CONNECTOR__STATE_CONNECTOR_PARAMS_H_
#define RRT_PLANNER__STATE_CONNECTOR__STATE_CONNECTOR_PARAMS_H_

#include <limits>

namespace state_space::state_connector {
/**
 * @brief State connector parameters holder
 */
struct StateConnectorParams {
  StateConnectorParams() = default;
  StateConnectorParams(bool allow_unknown_in,
                       unsigned char lethal_cost_in,
                       int max_extension_states_in)
      : allow_unknown(allow_unknown_in),
        lethal_cost(lethal_cost_in),
        max_extension_states(max_extension_states_in) {}

  bool allow_unknown{false};
  unsigned char lethal_cost{0};
  int max_extension_states{std::numeric_limits<int>::max()};
};
}  // namespace state_space::state_connector

#endif
