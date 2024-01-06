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

#ifndef RRT_PLANNER__PLANNER_CORE__SEARCH_TREE_H_
#define RRT_PLANNER__PLANNER_CORE__SEARCH_TREE_H_

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

namespace rrt_planner::planner_core {
/**
 * @brief Holder for expanded nodes of tree
 * @tparam NodeT
 */
template <typename NodeT>
class SearchTree {
 public:
  using NodePtr = NodeT*;
  using NodeVector = std::vector<NodePtr>;
  using EdgeT = std::pair<unsigned int, unsigned int>;
  using TreeT = std::vector<EdgeT>;

  /**
   * @brief Computes distance between states in state space associated with
   * given indexes
   */
  using DistanceGetter = std::function<double(unsigned int, unsigned int)>;

  SearchTree() = default;

  ~SearchTree() { clear(); }

  void clear() {
    root_node_ = nullptr;
    target_node_ = nullptr;
    tree_.clear();
  }

  void reserve(unsigned int size) { tree_.reserve(size); }

  void addVertex(const NodePtr& node) { tree_.push_back(node); }

  bool isNodeInTree(const NodePtr& node) {
    return std::find(tree_.begin(), tree_.end(), node) != tree_.end();
  }

  void setRootNode(const NodePtr& node) {
    root_node_ = node;
    addVertex(node);
  }

  void setTargetNode(const NodePtr& node) { target_node_ = node; }

  /**
   * @brief Returns closest node to state associated with given index
   * @param index Given index
   * @param distance_getter Computes distance between two states in state space
   * @return NodePtr
   */
  NodePtr getClosestNode(unsigned int index,
                         DistanceGetter& distance_getter) const {
    if (tree_.empty()) {
      return nullptr;
    }

    return *std::min_element(tree_.cbegin(), tree_.cend(),
                             [&](const auto& node1, const auto& node2) {
                               return distance_getter(node1->getIndex(),
                                                      index) <
                                      distance_getter(node2->getIndex(), index);
                             });
  }

  /**
   * @brief Returns vector of nodes in neighborhood of state associated with
   * given index
   * @param index Given index
   * @param near_nodes Filled by method
   * @param distance_getter Computes distance between two states in state space
   * @param near_distance Defines neighborhood of nodes
   */
  void getNearNodes(unsigned int index, NodeVector& near_nodes,
                    DistanceGetter& distance_getter, double near_distance) {
    near_nodes.clear();

    if (tree_.empty()) {
      return;
    }

    std::copy_if(tree_.cbegin(), tree_.cend(), std::back_inserter(near_nodes),
                 [&](const NodePtr& node) {
                   return distance_getter(node->getIndex(), index) <=
                          near_distance;
                 });
  }

  /**
   * @brief Returns search tree as vector of pairs of indexes where each pair
   * represents parent-child relation in search tree
   * @return TreeT
   */
  TreeT getSearchTree() const {
    TreeT tree;
    tree.reserve(tree_.size());

    std::for_each(tree_.cbegin(), tree_.cend(), [&](const auto& node) {
      if (node != nullptr && node->parent != nullptr) {
        tree.emplace_back(node->getIndex(), node->parent->getIndex());
      }
    });

    return tree;
  }

 protected:
  // Root node pointer
  NodePtr root_node_{nullptr};
  // Target node pointer
  NodePtr target_node_{nullptr};
  // Holder for nodes in tree
  NodeVector tree_;
};

}  // namespace rrt_planner::planner_core

#endif
