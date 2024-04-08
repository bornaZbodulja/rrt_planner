/**
 * @file search_tree.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Search tree implementation
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_TREE_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_TREE_H_

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

#include "rrt_planner/planner_core/planner_entities/node.h"

namespace rrt_planner::planner_core::planner_entities {
/**
 * @brief Holder for expanded nodes of tree
 * @tparam StateT State template
 */
template <typename StateT>
class SearchTree {
 public:
  using NodeT = Node<StateT>;
  /**
   * @brief Computes distance between nodes in state space associated with
   * given states
   */
  using DistanceGetter = std::function<double(const StateT&, const StateT&)>;

  SearchTree(DistanceGetter&& distance_getter)
      : distance_getter_(std::move(distance_getter)) {}

  ~SearchTree() = default;

  void clear() { tree_.clear(); }

  void reserve(std::size_t size) { tree_.reserve(size); }

  void addVertex(NodeT* node) { tree_.push_back(node); }

  bool isNodeInTree(const NodeT* node) {
    return std::find(tree_.begin(), tree_.end(), node) != tree_.end();
  }

  /**
   * @brief Returns closest node to given state
   * @param state Given state
   * @return NodeT* Pointer to closest node
   */
  NodeT* getClosestNode(const StateT& state) const {
    if (tree_.empty()) {
      return nullptr;
    }

    return *std::min_element(
        tree_.cbegin(), tree_.cend(),
        [this, &state](const NodeT* node1, const NodeT* node2) {
          return distance_getter_(node1->getState(), state) <
                 distance_getter_(node2->getState(), state);
        });
  }

  /**
   * @brief Returns vector of nodes in neighborhood of given state
   * @param state Given state
   * @param near_distance Defines neighborhood of nodes
   */
  std::vector<NodeT*> getNearNodes(const StateT& state,
                                   double near_distance) const {
    std::vector<NodeT*> near_nodes{};

    if (tree_.empty()) {
      return {};
    }

    std::copy_if(tree_.cbegin(), tree_.cend(), std::back_inserter(near_nodes),
                 [this, &state, &near_distance](const NodeT* node) {
                   return distance_getter_(node->getState(), state) <=
                          near_distance;
                 });

    return near_nodes;
  }

  /**
   * @brief Returns search tree as vector of pairs of states where each pair
   * represents parent-child relation in search tree
   * @return TreeT
   */
  std::vector<std::pair<StateT, StateT>> getTreeEdges() const {
    std::vector<std::pair<StateT, StateT>> edges;
    edges.reserve(tree_.size());

    std::for_each(tree_.cbegin(), tree_.cend(), [&](const NodeT* node) {
      if (node != nullptr && node->parent != nullptr) {
        edges.emplace_back(node->parent->getState(), node->getState());
      }
    });

    return edges;
  }

 private:
  // Holder for nodes in tree
  std::vector<NodeT*> tree_;
  // Distance getter
  DistanceGetter distance_getter_;
};
}  // namespace rrt_planner::planner_core::planner_entities

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_TREE_H_
