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
      goal_tree_(SearchTree<NodeT>()) {
  UpdateMotionModel(motion_model);
  UpdateSearchInfo(search_info);
  UpdateCollisionChecker(collision_checker);

  // TODO: Move this to utils
  gen = std::minstd_rand(std::random_device{}());
  dist = std::uniform_real_distribution<double>(0.0, 1.0);
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

  const int state_space_size = size_x_ * size_y_ * dim_3_;
  ClearGraph();
  ReserveGraph(state_space_size);
  NodeT::InitializeMotionModel(size_x_, dim_3_, search_info_, motion_model_);
}

template <>
void RRTStar<Node2D>::InitializeStateSpace(const unsigned int& size_x,
                                           const unsigned int& size_y,
                                           const unsigned int& /*dim_3*/) {
  size_x_ = size_x;
  size_y_ = size_y;
  dim_3_ = 1;

  const int state_space_size = size_x_ * size_y_;
  ClearGraph();
  ReserveGraph(state_space_size);
  Node2D::InitializeMotionModel(size_x_, search_info_, motion_model_);
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
  const double& cost_penalty = search_info_.cost_penalty;
  const double& near_distance = search_info_.near_distance;
  const unsigned int& state_space_size = size_x_ * size_y_ * dim_3_;

  const auto start_time = steady_clock::now();
  duration<double> elapsed_time = milliseconds(0);

  auto start_index = start_->GetIndex();
  auto goal_index = goal_->GetIndex();

  InitializeSearch(max_iterations, cost_penalty, near_distance);

  // Preallocating variables
  int iterations{0};
  unsigned int new_index{0};
  NodePtr new_node{nullptr};
  NodePtr closest_node{nullptr};
  NodeVector near_nodes{};
  auto& current_target_index = goal_index;
  bool expanding_start_tree{true};
  bool tree_expansion_res{false};
  bool tree_connection_res{false};

  path.clear();

  ROS_INFO("Start index: %d, goal index: %d", start_index, goal_index);

  while (iterations < max_iterations &&
         elapsed_time.count() < max_planning_time) {
    // 1) Get new index in state space
    new_index =
        GetNewIndex(target_bias, current_target_index, state_space_size);
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
        expanding_start_tree ? ConnectTrees(new_node, closest_node, goal_tree_,
                                            path, lethal_cost, allow_unknown)
                             : ConnectTrees(new_node, closest_node, start_tree_,
                                            path, lethal_cost, allow_unknown);

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
                                      const double& cost_penalty,
                                      const double& near_distance) {
  start_tree_.Clear();
  start_tree_.Reserve(size);
  start_tree_.SetRootNode(start_);
  start_tree_.SetTargetNode(goal_);

  goal_tree_.Clear();
  goal_tree_.Reserve(size);
  goal_tree_.SetRootNode(goal_);
  goal_tree_.SetTargetNode(start_);

  SearchTree<NodeT>::near_distance = near_distance;
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
  const auto connection_res = closest_node->ConnectNode(
      index, collision_checker_, lethal_cost, allow_unknown, edge_length);

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

  // New node coordinates
  // TODO: Remove this in future if possible
  const auto new_node_coordinates = new_node->GetCoordinates();

  // Set costmap cost for new node
  new_node->SetCost(static_cast<double>(collision_checker_->GetCost(
      new_node_coordinates.x, new_node_coordinates.y)));

  tree.GetNearNodes(new_node_index, near_nodes);

  NodePtr best_parent =
      ChooseParent(new_node, near_nodes, lethal_cost, allow_unknown);

  double accumulated_cost;

  if (best_parent == nullptr) {
    new_node->SetParent(closest_node);
    accumulated_cost = closest_node->GetAccumulatedCost() +
                       closest_node->GetTraversalCost(new_node);
    new_node->SetAccumulatedCost(accumulated_cost);
  } else {
    new_node->SetParent(best_parent);
    accumulated_cost = best_parent->GetAccumulatedCost() +
                       best_parent->GetTraversalCost(new_node);
    new_node->SetAccumulatedCost(accumulated_cost);
  }

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

  auto connection_res = new_node->ConnectNode(
      closest_node->GetIndex(), collision_checker_, lethal_cost, allow_unknown);

  if (!connection_res.has_value()) {
    return false;
  }

  auto first_segment = new_node->BackTracePath();
  auto second_segment = closest_node->BackTracePath();

  std::reverse(first_segment.begin(), first_segment.end());
  std::move(first_segment.begin(), first_segment.end(),
            std::back_inserter(path));
  std::move(second_segment.begin(), second_segment.end(),
            std::back_inserter(path));

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
              ->ConnectNode(new_node->GetIndex(), collision_checker_,
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
unsigned int RRTStar<NodeT>::GetNewIndex(const double& target_bias,
                                         const unsigned int& target_index,
                                         const unsigned int& state_space_size) {
  const double r = dist(gen);

  if (r <= target_bias) {
    return target_index;
  }

  return GenerateRandomIndex(state_space_size);
}

template <typename NodeT>
unsigned int RRTStar<NodeT>::GenerateRandomIndex(
    const unsigned int& state_space_size) {
  return std::experimental::randint(static_cast<unsigned int>(0),
                                    state_space_size);
}

// Instantiate algorithm for the supported template types
template class RRTStar<Node2D>;
template class RRTStar<NodeHybrid>;
