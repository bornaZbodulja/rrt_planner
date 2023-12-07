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
class ROSStateConnectorFactory {
 public:
  template <typename StateT>
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  template <typename StateT>
  using StateConnectorPtr = std::unique_ptr<StateConnectorT<StateT>>;
  using State2D = state_space::state_space_2d::State2D;
  using StateConnector2DT = state_space::state_connector_2d::StateConnector2D;
  using StateHybrid = state_space::state_space_hybrid::StateHybrid;
  using StateSpaceHybrid = state_space::state_space_hybrid::StateSpaceHybrid;
  using StateSpaceHybridPtr = std::shared_ptr<StateSpaceHybrid>;
  using StateConnectorHybridT =
      state_space::state_connector_hybrid::StateConnectorHybrid;
  using StateConnectorParamsT =
      state_space::state_connector::StateConnectorParams;

  ROSStateConnectorFactory() = delete;

  /**
   * @brief
   * @param collision_checker
   * @param costmap_resolution
   * @param nh
   * @return StateConnectorPtr<State2D>
   */
  static StateConnectorPtr<State2D> create2DStateConnector(
      const CollisionCheckerPtr& collision_checker, double costmap_resolution,
      ros::NodeHandle* nh) {
    auto&& connector_params =
        rrt_planner::param_loader::loadStateConnectorParams(nh,
                                                            costmap_resolution);
    return std::make_unique<StateConnector2DT>(connector_params,
                                               collision_checker);
  }

  /**
   * @brief 
   * @param state_space 
   * @param collision_checker 
   * @param costmap_resolution 
   * @param nh 
   * @return StateConnectorPtr<StateHybrid> 
   */
  static StateConnectorPtr<StateHybrid> createHybridStateConnector(
      const StateSpaceHybridPtr& state_space,
      const CollisionCheckerPtr& collision_checker, double costmap_resolution,
      ros::NodeHandle* nh) {
    auto&& connector_params =
        rrt_planner::param_loader::loadStateConnectorParams(nh,
                                                            costmap_resolution);
    auto&& hybrid_model =
        rrt_planner::param_loader::loadHybridModel(nh, costmap_resolution);
    return std::make_unique<StateConnectorHybridT>(
        state_space, hybrid_model, connector_params, collision_checker);
  }
};
}  // namespace rrt_planner::ros_factory

#endif
