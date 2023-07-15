/**
 * @file rrt_star.cpp
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT* algorithm implementation
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "rrt_planner/rrt_star.h"

#include "nav_utils/geometry_utils.h"

using namespace rrt_planner;
using namespace std::chrono;

template <typename NodeT>
RRTStar<NodeT>::RRTStar(const MotionModel& motion_model,
                        const SearchInfo& search_info,
                        const CollisionCheckerPtr& collision_checker)
    : start_(nullptr),
      goal_(nullptr),
      graph_(SearchGraph<NodeT>()),
      start_tree_(SearchTree<NodeT>()),
      goal_tree_(SearchTree<NodeT>()),
      sampler_(StateSampler<NodeT>(collision_checker)) {
  UpdateMotionModel(motion_model);
  UpdateSearchInfo(search_info);
  UpdateCollisionChecker(collision_checker);
}

template <typename NodeT>
RRTStar<NodeT>::~RRTStar() {
  start_ = nullptr;
  goal_ = nullptr;
}

template <typename NodeT>
void RRTStar<NodeT>::InitializeStateSpace(const unsigned int& size_x,
                                          const unsigned int& size_y,
                                          const unsigned int& dim_3) {
  size_x_ = size_x;
  size_y_ = size_y;
  dim_3_ = dim_3;

  ClearGraph();
  ReserveGraph(search_info_.max_expansion_iterations);
  NodeT::InitializeMotionTable(size_x_, dim_3_, search_info_);
}

template <>
void RRTStar<Node2D>::InitializeStateSpace(const unsigned int& size_x,
                                           const unsigned int& size_y,
                                           const unsigned int& /*dim_3*/) {
  size_x_ = size_x;
  size_y_ = size_y;
  dim_3_ = 1;

  ClearGraph();
  ReserveGraph(search_info_.max_expansion_iterations);
  Node2D::InitializeMotionTable(size_x_, search_info_);
}

template <typename NodeT>
void RRTStar<NodeT>::SetStart(const unsigned int& mx, const unsigned int& my,
                              const unsigned int& dim_3) {
  const auto start_index = NodeT::GetIndex(mx, my, dim_3);
  start_ = AddToGraph(start_index);
  start_->SetAccumulatedCost(0.0);
  start_->SetCost(collision_checker_->GetCost(mx, my));
  start_->Visited();
}

template <>
void RRTStar<Node2D>::SetStart(const unsigned int& mx, const unsigned int& my,
                               const unsigned int& /*dim_3*/) {
  const auto start_index = Node2D::GetIndex(mx, my);
  start_ = AddToGraph(start_index);
  start_->SetAccumulatedCost(0.0);
  start_->SetCost(collision_checker_->GetCost(mx, my));
  start_->Visited();
}

template <typename NodeT>
void RRTStar<NodeT>::SetGoal(const unsigned int& mx, const unsigned int& my,
                             const unsigned int& dim_3) {
  const auto goal_index = NodeT::GetIndex(mx, my, dim_3);
  goal_ = AddToGraph(goal_index);
  goal_->SetAccumulatedCost(0.0);
  goal_->SetCost(collision_checker_->GetCost(mx, my));
  goal_->Visited();
}

template <>
void RRTStar<Node2D>::SetGoal(const unsigned int& mx, const unsigned int& my,
                              const unsigned int& /*dim_3*/) {
  const auto goal_index = Node2D::GetIndex(mx, my);
  goal_ = AddToGraph(goal_index);
  goal_->SetAccumulatedCost(0.0);
  goal_->SetCost(collision_checker_->GetCost(mx, my));
  goal_->Visited();
}

template <typename NodeT>
bool RRTStar<NodeT>::CreatePath(CoordinatesVector& path) {
  const int& max_iterations = search_info_.max_expansion_iterations;
  const double& max_planning_time = search_info_.max_planning_time;
  const double& target_bias = search_info_.target_bias;
  const bool& allow_unknown = search_info_.allow_unknown;
  const bool& rewire_tree = search_info_.rewire_tree;
  const unsigned char& lethal_cost = search_info_.lethal_cost;
  const int& edge_length = search_info_.edge_length;
  const double& near_distance = search_info_.near_distance;
  const double& connect_trees_max_length =
      search_info_.connect_trees_max_length;
  const unsigned int& state_space_size = size_x_ * size_y_ * dim_3_;
  const double& rgd_increment_step = search_info_.rgd_increment_step;
  const unsigned char rgd_stop_cost = search_info_.rgd_stop_cost;
  const int& rgd_iterations = search_info_.rgd_iterations;

  const auto start_time = steady_clock::now();
  duration<double> elapsed_time = milliseconds(0);

  auto start_index = start_->GetIndex();
  auto goal_index = goal_->GetIndex();

  InitializeSearch(max_iterations, near_distance);
  sampler_.UpdateSamplerParams(target_bias, state_space_size,
                               rgd_increment_step, rgd_stop_cost,
                               rgd_iterations);

  // Preallocating variables
  int iterations{0};
  unsigned int new_index;
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodeVector near_nodes;
  auto& current_target_index{goal_index};
  bool expanding_start_tree{true};
  bool tree_expansion_res{false};
  bool tree_connection_res{false};

  path.clear();

  while (iterations < max_iterations &&
         elapsed_time.count() < max_planning_time) {
    // 1) Get new index in state space
    new_index = sampler_(current_target_index);
    iterations++;

    // 2) Extend search tree with new index
    tree_expansion_res =
        expanding_start_tree
            ? ExtendTree(new_index, start_tree_, new_node, closest_node,
                         near_nodes, edge_length, rewire_tree, lethal_cost,
                         allow_unknown)
            : ExtendTree(new_index, goal_tree_, new_node, closest_node,
                         near_nodes, edge_length, rewire_tree, lethal_cost,
                         allow_unknown);

    // 3) Try connecting searches trees
    tree_connection_res =
        expanding_start_tree
            ? ConnectTrees(new_node, closest_node, goal_tree_, path,
                           connect_trees_max_length, lethal_cost, allow_unknown)
            : ConnectTrees(new_node, closest_node, start_tree_, path,
                           connect_trees_max_length, lethal_cost,
                           allow_unknown);

    if (tree_expansion_res && tree_connection_res) {
      ROS_INFO(
          "RRT planner found path, used iterations %d/%d, "
          "planning time: %.3f.",
          iterations, max_iterations, elapsed_time.count());

      if (!expanding_start_tree) {
        std::reverse(path.begin(), path.end());
      }

      return true;
    }

    // 4) Switching to other tree
    expanding_start_tree = !expanding_start_tree;
    current_target_index = expanding_start_tree ? goal_index : start_index;

    elapsed_time = steady_clock::now() - start_time;
  }

  ROS_INFO(
      "RRT* unable to find path, used iterations %d/%d, "
      "planning time: %.3f.",
      iterations, max_iterations, elapsed_time.count());

  return false;
}

template <typename NodeT>
void RRTStar<NodeT>::InitializeSearch(const unsigned int& size,
                                      const double& near_distance) {
  start_tree_.Clear();
  start_tree_.Reserve(size);
  start_tree_.InitializeSearchTree(start_, goal_, near_distance);

  goal_tree_.Clear();
  goal_tree_.Reserve(size);
  goal_tree_.InitializeSearchTree(goal_, start_, near_distance);
}

template <typename NodeT>
bool RRTStar<NodeT>::ExtendTree(const unsigned int& index,
                                SearchTree<NodeT>& tree, NodePtr& new_node,
                                NodePtr& closest_node, NodeVector& near_nodes,
                                const int& edge_length, const bool& rewire_tree,
                                const unsigned char& lethal_cost,
                                const bool& allow_unknown) {
  new_node = nullptr;
  closest_node = nullptr;
  unsigned int new_node_index;

  // Getting closest node
  closest_node = tree.GetClosestNode(index);

  if (closest_node == nullptr) {
    return false;
  }

  // Try connecting closest node with the indexed node
  const auto connection_res =
      closest_node->ExtendNode(NodeT::GetCoordinates(index), collision_checker_,
                               lethal_cost, allow_unknown, edge_length);

  if (!connection_res.has_value()) {
    new_node = nullptr;
    return false;
  }

  new_node_index = connection_res.value();

  // Get new node with index
  new_node = graph_.GetNode(new_node_index);

  // Returning if node already in tree
  if (tree.IsNodeInTree(new_node)) {
    new_node = nullptr;
    return false;
  }

  // Set costmap cost for new node
  new_node->SetCost(static_cast<double>(collision_checker_->GetCost(
      new_node->coordinates.x, new_node->coordinates.y)));

  tree.GetNearNodes(new_node_index, near_nodes);

  NodePtr best_parent =
      ChooseParent(new_node, near_nodes, lethal_cost, allow_unknown);

  double accumulated_cost;

  if (best_parent == nullptr) {
    best_parent = closest_node;
  }

  best_parent = BackTracking(new_node, best_parent, lethal_cost, allow_unknown);

  new_node->parent = best_parent;
  accumulated_cost = best_parent->GetAccumulatedCost() +
                     best_parent->GetTraversalCost(new_node);
  new_node->SetAccumulatedCost(accumulated_cost);

  new_node->Visited();
  tree.AddVertex(new_node);

  if (rewire_tree) {
    // Rewire tree around new node
    tree.RewireTree(new_node, near_nodes, collision_checker_, lethal_cost,
                    allow_unknown);
  }

  return true;
}

template <typename NodeT>
bool RRTStar<NodeT>::ConnectTrees(NodePtr& new_node, NodePtr& closest_node,
                                  SearchTree<NodeT>& second_tree,
                                  CoordinatesVector& path,
                                  const double& connect_trees_max_length,
                                  const unsigned char& lethal_cost,
                                  const bool& allow_unknown) {
  if (new_node == nullptr) {
    return false;
  }

  closest_node = nullptr;
  closest_node = second_tree.GetClosestNode(new_node->GetIndex());

  if (closest_node == nullptr) {
    return false;
  }

  if (nav_utils::GetEuclideanDistance<typename NodeT::Coordinates>(
          new_node->coordinates, closest_node->coordinates) >
      connect_trees_max_length) {
    return false;
  }

  auto connection_res =
      new_node->ExtendNode(closest_node->coordinates, collision_checker_,
                           lethal_cost, allow_unknown);

  if (!connection_res.has_value()) {
    return false;
  }

  auto first_segment = new_node->BackTracePath();
  auto second_segment = closest_node->BackTracePath();
  NodeVector node_path;

  std::reverse(first_segment.begin(), first_segment.end());
  std::move(first_segment.begin(), first_segment.end(),
            std::back_inserter(node_path));
  std::move(second_segment.begin(), second_segment.end(),
            std::back_inserter(node_path));

  path = PreparePath(node_path);

  return true;
}

template <typename NodeT>
typename RRTStar<NodeT>::NodePtr RRTStar<NodeT>::ChooseParent(
    NodePtr& new_node, NodeVector& near_nodes, const unsigned char& lethal_cost,
    const bool& allow_unknown) {
  if (near_nodes.empty()) {
    return nullptr;
  }

  NodePtr best_parent{nullptr};
  double min_cost{std::numeric_limits<double>::max()};
  double current_cost{0};
  bool smaller_cost_parent_found{false};
  bool connection_valid{false};

  for (auto& near_node : near_nodes) {
    current_cost =
        near_node->GetAccumulatedCost() + near_node->GetTraversalCost(new_node);

    smaller_cost_parent_found = current_cost < min_cost;

    if (smaller_cost_parent_found) {
      connection_valid =
          near_node
              ->ExtendNode(new_node->coordinates, collision_checker_,
                           lethal_cost, allow_unknown)
              .has_value();
    }

    if (smaller_cost_parent_found && connection_valid) {
      min_cost = current_cost;
      best_parent = near_node;
    }
  }

  return best_parent;
}

template <typename NodeT>
typename RRTStar<NodeT>::NodePtr RRTStar<NodeT>::BackTracking(
    NodePtr& new_node, NodePtr& parent_node, const unsigned char& lethal_cost,
    const bool& allow_unknown) {
  NodePtr current_anc = parent_node;
  NodePtr current_parent = parent_node;
  bool connection_valid{false};

  while (current_anc->parent != nullptr) {
    current_anc = current_anc->parent;
    connection_valid =
        current_anc
            ->ExtendNode(new_node->coordinates, collision_checker_, lethal_cost,
                         allow_unknown)
            .has_value();
    if (connection_valid) {
      current_parent = current_anc;
    } else {
      return current_parent;
    }
  }

  return current_parent;
}

template <typename NodeT>
typename RRTStar<NodeT>::CoordinatesVector RRTStar<NodeT>::PreparePath(
    const NodeVector& path) {
  CoordinatesVector coordinates_path, segment;
  NodePtr current_node, parent_node;

  for (size_t i = 0; i < path.size() - 1; i++) {
    current_node = path.at(i);
    parent_node = path.at(i + 1);
    segment = parent_node->ConnectNode(current_node);
    std::reverse(segment.begin(), segment.end());
    std::move(segment.begin(), segment.end(),
              std::back_inserter(coordinates_path));
  }
  return coordinates_path;
}

// Instantiate algorithm for the supported template types
template class RRTStar<Node2D>;
template class RRTStar<NodeHybrid>;
