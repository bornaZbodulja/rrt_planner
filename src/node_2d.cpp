/**
 * @file node_2d.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/node_2d.h"

using namespace rrt_planner;

Node2D::Node2D(const unsigned int& index)
    : parent(nullptr),
      coordinates(GetCoordinates(index)),
      index_(index),
      visited_(false),
      cell_cost_(std::numeric_limits<double>::quiet_NaN()),
      accumulated_cost_(std::numeric_limits<double>::max()) {}

Node2D::~Node2D() { parent = nullptr; }

void Node2D::Reset() {
  parent = nullptr;
  visited_ = false;
  cell_cost_ = std::numeric_limits<double>::quiet_NaN();
  accumulated_cost_ = std::numeric_limits<double>::max();
}

bool Node2D::IsNodeValid(const CollisionCheckerPtr& collision_checker,
                         const unsigned char& lethal_cost,
                         const bool& allow_unknown) {
  return !collision_checker->PointInCollision(coordinates.x, coordinates.y,
                                              lethal_cost, allow_unknown);
}

double Node2D::GetTraversalCost(const NodePtr& child) {
  const double normalized_cost = child->GetCost() / 253.0;

  return CoordinatesDistance(this->coordinates, child->coordinates) +
         motion_table.cost_travel_multiplier * normalized_cost;
}

std::optional<unsigned int> Node2D::ConnectNode(
    const unsigned int& index, const CollisionCheckerPtr& collision_checker,
    const unsigned char& lethal_cost, const bool& allow_unknown,
    const int& edge_length) {
  const Coordinates child_coordinates = GetCoordinates(index);

  auto line =
      LineIteratorT(this->coordinates.x, this->coordinates.y,
                    child_coordinates.x, child_coordinates.y, edge_length);
  bool line_point_in_collision{false};

  for (; line.IsValid(); line.Advance()) {
    line_point_in_collision = collision_checker->PointInCollision(
        line.GetCurrentX(), line.GetCurrentY(), lethal_cost, allow_unknown);
    if (line_point_in_collision) {
      return {};
    }
  }

  unsigned int connected_node_index;
  collision_checker->GetIndex(static_cast<unsigned int>(line.GetPreviousX()),
                              static_cast<unsigned int>(line.GetPreviousY()),
                              connected_node_index);
  return std::make_optional(connected_node_index);
}

void Node2D::RewireNode(const NodePtr& parent, const double& accumulated_cost) {
  this->parent = parent;
  accumulated_cost_ = accumulated_cost;
}

Node2D::CoordinatesVector Node2D::BackTracePath() {
  CoordinatesVector path;
  NodePtr current_node = this;

  while (current_node != nullptr) {
    path.push_back(current_node->coordinates);
    current_node = current_node->parent;
  }

  return path;
}

void Node2D::InitializeMotionModel(const unsigned int& size_x_in,
                                   const SearchInfo& search_info,
                                   const MotionModel& motion_model) {
  motion_table.Initialize(size_x_in, search_info, motion_model);
}
