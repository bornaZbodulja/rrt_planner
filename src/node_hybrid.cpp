/**
 * @file node_hybrid.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/node_hybrid.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

using namespace rrt_planner;

void HybridMotionTable::Initialize(const unsigned int& size_x_in,
                                   const unsigned int& angle_bin_size_in,
                                   const SearchInfo& search_info,
                                   const MotionModel& motion_model_in) {
  size_x = size_x_in;
  angle_bin_size = angle_bin_size_in;
  motion_model = motion_model_in;
  min_turning_radius = search_info.min_turning_radius;
  cost_travel_multiplier = search_info.cost_penalty;
  angle_bin = 2.0 * M_PI / angle_bin_size;

  if (motion_model == MotionModel::DUBINS) {
    state_space =
        std::make_unique<ompl::base::DubinsStateSpace>(min_turning_radius);
  } else if (motion_model == MotionModel::REEDS_SHEPP) {
    state_space =
        std::make_unique<ompl::base::ReedsSheppStateSpace>(min_turning_radius);
  }
}

NodeHybrid::NodeHybrid(const unsigned int& index)
    : index_(index),
      visited_(false),
      parent_(nullptr),
      coordinates_(GetCoordinates(index)),
      cell_cost_(std::numeric_limits<double>::quiet_NaN()),
      accumulated_cost_(std::numeric_limits<double>::max()) {}

NodeHybrid::~NodeHybrid() { parent_ = nullptr; }

void NodeHybrid::Reset() {
  parent_ = nullptr;
  visited_ = false;
  cell_cost_ = std::numeric_limits<double>::quiet_NaN();
  accumulated_cost_ = std::numeric_limits<double>::max();
}

double NodeHybrid::GetTraversalCost(const NodePtr& child) {
  const double normalized_cost = child->GetCost() / 253.0;
  const Coordinates A = GetCoordinates();
  const Coordinates B = child->GetCoordinates();

  return expander->GetAnalyticPathLength(A, B, this) +
         motion_table.cost_travel_multiplier * normalized_cost;
}

bool NodeHybrid::IsNodeValid(const CollisionCheckerPtr& collision_checker,
                             const unsigned char& lethal_cost,
                             const bool& allow_unknown) {
  const double angle = motion_table.GetAngleFromBin(coordinates_.theta);
  return !collision_checker->PoseInCollision(
      static_cast<unsigned int>(coordinates_.x),
      static_cast<unsigned int>(coordinates_.y), angle, lethal_cost,
      allow_unknown);
}

std::optional<unsigned int> NodeHybrid::ConnectNode(
    const unsigned int& index, const CollisionCheckerPtr& collision_checker,
    const unsigned char& lethal_cost, const bool& allow_unknown,
    const int& edge_length) {
  const auto A = GetCoordinates();
  const auto B = GetCoordinates(index);
  // TODO: Possibly remove this, update collision checker only when starting
  // planning
  expander->UpdateCollisionChecker(collision_checker);

  const auto result = expander->TryAnalyticExpansion(
      A, B, this, lethal_cost, allow_unknown, edge_length);

  if (!result.has_value()) {
    return {};
  }

  return std::make_optional(
      GetIndex(result.value().x, result.value().y, result.value().theta));
}

void NodeHybrid::RewireNode(const NodePtr& parent,
                            const double& accumulated_cost) {
  SetParent(parent);
  SetAccumulatedCost(accumulated_cost);
}

NodeHybrid::CoordinatesVector NodeHybrid::BackTracePath() {
  CoordinatesVector path;
  NodePtr current_node = this;

  while (current_node != nullptr) {
    path.push_back(current_node->GetCoordinates());
    current_node = current_node->GetParent();
  }

  return path;
}

void NodeHybrid::InitializeMotionModel(const unsigned int& size_x_in,
                                       const unsigned int& angle_bin_size_in,
                                       const SearchInfo& search_info,
                                       const MotionModel& motion_model) {
  motion_table.Initialize(size_x_in, angle_bin_size_in, search_info,
                          motion_model);
  expander = std::make_unique<AnalyticExpansion<NodeHybrid>>();
  expander->UpdateMotionModel(motion_model);
}
