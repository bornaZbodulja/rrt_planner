/**
 * @file ros_state_connector_factory.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__ROS_FACTORY__ROS_STATE_CONNECTOR_FACTORY_H_
#define RRT_PLANNER__ROS_FACTORY__ROS_STATE_CONNECTOR_FACTORY_H_

#include <nav_utils/collision_checker.h>
#include <ros/node_handle.h>

#include <memory>

#include "rrt_planner/param_loader/connector_params_loader.h"
#include "rrt_planner/param_loader/hybrid_model_loader.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_connector/state_connector_params.h"
#include "state_space/state_connector_2d/state_connector_2d.h"
#include "state_space/state_connector_hybrid/state_connector_hybrid.h"
#include "state_space/state_space_2d/state_2d.h"
#include "state_space/state_space_hybrid/state_hybrid.h"
#include "state_space/state_space_hybrid/state_space_hybrid.h"

namespace rrt_planner::ros_factory {

inline std::unique_ptr<state_space::state_connector::StateConnector<
    state_space::state_space_2d::State2D>>
create2DStateConnector(
    ros::NodeHandle* nh,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  state_space::state_connector::StateConnectorParams connector_params =
      rrt_planner::param_loader::loadStateConnectorParams(
          nh, collision_checker->getMapResolution());
  return std::make_unique<state_space::state_connector_2d::StateConnector2D>(
      std::move(connector_params), collision_checker);
}

inline std::unique_ptr<state_space::state_connector::StateConnector<
    state_space::state_space_hybrid::StateHybrid>>
createHybridStateConnector(
    ros::NodeHandle* nh,
    const std::shared_ptr<state_space::state_space_hybrid::StateSpaceHybrid>&
        state_space,
    const std::shared_ptr<nav_utils::CollisionChecker>& collision_checker) {
  state_space::state_connector::StateConnectorParams connector_params =
      rrt_planner::param_loader::loadStateConnectorParams(
          nh, collision_checker->getMapResolution());
  state_space::state_connector_hybrid::HybridModel hybrid_model =
      rrt_planner::param_loader::loadHybridModel(
          nh, collision_checker->getMapResolution());
  return std::make_unique<
      state_space::state_connector_hybrid::StateConnectorHybrid>(
      std::move(hybrid_model), std::move(connector_params), state_space,
      collision_checker);
}
}  // namespace rrt_planner::ros_factory

#endif
