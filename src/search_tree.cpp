/**
 * @file search_tree.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/search_tree.h"

#include "nav_utils/geometry_utils.h"

using namespace rrt_planner;

template <typename NodeT>
SearchTree<NodeT>::~SearchTree() {
  root_node_ = nullptr;
  target_node_ = nullptr;
}

template <typename NodeT>
void SearchTree<NodeT>::InitializeSearchTree(const NodePtr& root_node,
                                             const NodePtr& target_node,
                                             const double& near_distance) {
  SetRootNode(root_node);
  SetTargetNode(target_node);
  near_distance_ = near_distance;
}

template <typename NodeT>
bool SearchTree<NodeT>::IsNodeInTree(const NodePtr& node) {
  const auto node_in_tree =
      std::find(tree_.begin(), tree_.end(), node) != tree_.end();
  return node_in_tree;
}

template <typename NodeT>
typename SearchTree<NodeT>::NodePtr SearchTree<NodeT>::GetClosestNode(
    const unsigned int& index) {
  if (tree_.empty()) {
    return nullptr;
  }

  auto smallest_distance{std::numeric_limits<double>::max()};
  double distance{0.0};

  NodeIterator nearest_it;

  const Coordinates new_node_coordinates = NodeT::GetCoordinates(index);

  for (auto current_it = tree_.begin(); current_it < tree_.end();
       current_it++) {
    distance = NodeT::CoordinatesDistance(new_node_coordinates,
                                          (*current_it)->coordinates);
    // If computed distance smaller than smallest, update nearest iterator and
    // smallest distance
    if (distance < smallest_distance) {
      nearest_it = current_it;
      smallest_distance = distance;
    }
  }

  return (*nearest_it);
}

template <typename NodeT>
void SearchTree<NodeT>::GetNearNodes(const unsigned int& index,
                                     NodeVector& near_nodes) {
  near_nodes.clear();

  if (tree_.empty()) {
    return;
  }

  const Coordinates new_node_coordinates = NodeT::GetCoordinates(index);
  double distance{0.0};
  double near_distance_squared = std::pow(near_distance_, 2);

  for (auto current_it = tree_.cbegin(); current_it < tree_.cend();
       current_it++) {
    distance = nav_utils::GetEuclideanDistanceSquared<typename NodeT::Coordinates>(
        new_node_coordinates, (*current_it)->coordinates);
    // If computed distance smaller than near distance, adding node to near
    // nodes vector
    if (distance <= near_distance_squared) {
      near_nodes.push_back(*current_it);
    }
  }
}

template <typename NodeT>
void SearchTree<NodeT>::RewireTree(NodePtr& new_node, NodeVector& near_nodes,
                                   const CollisionCheckerPtr& collision_checker,
                                   const unsigned char& lethal_cost,
                                   const bool& allow_unknown) {
  if (near_nodes.empty() && new_node == nullptr) {
    return;
  }

  bool smaller_cost_approach_found{false};
  bool connection_valid{false};
  double new_approach_cost;

  for (auto& near_node : near_nodes) {
    // New approach cost
    new_approach_cost =
        new_node->GetAccumulatedCost() + new_node->GetTraversalCost(near_node);

    smaller_cost_approach_found =
        near_node->GetAccumulatedCost() > new_approach_cost;

    // Found smaller cost approach to node, checking connection validity
    if (smaller_cost_approach_found) {
      connection_valid =
          new_node
              ->ExtendNode(near_node->coordinates, collision_checker,
                           lethal_cost, allow_unknown)
              .has_value();
    }

    // If new approach cost lower than current cost and connection is valid,
    // rewire node
    if (smaller_cost_approach_found && connection_valid) {
      near_node->RewireNode(new_node, new_approach_cost);
    }
  }
}

template <typename NodeT>
typename SearchTree<NodeT>::TreeMsg SearchTree<NodeT>::TreeToMsg() {
  TreeMsg msg;
  msg.reserve(tree_.size());

  for (const auto& node : tree_) {
    if (node != nullptr && node->parent != nullptr) {
      msg.emplace_back(node->coordinates, node->parent->coordinates);
    }
  }

  return msg;
}

// Instantiate algorithm for the supported template types
template class SearchTree<Node2D>;
template class SearchTree<NodeHybrid>;
