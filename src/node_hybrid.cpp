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

using namespace rrt_planner;

NodeHybrid::NodeHybrid(const unsigned int& index)
    : index_(index),
      visited_(false),
      parent_(nullptr),
      cell_cost_(std::numeric_limits<double>::quiet_NaN()),
      accumulated_cost_(std::numeric_limits<double>::max()),
      coordinates_(GetCoordinates(index)) {}

NodeHybrid::~NodeHybrid() { parent_ = nullptr; }

void NodeHybrid::Reset() {
  parent_ = nullptr;
  visited_ = false;
  cell_cost_ = std::numeric_limits<double>::quiet_NaN();
  accumulated_cost_ = std::numeric_limits<double>::max();
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
