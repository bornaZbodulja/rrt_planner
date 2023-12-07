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
 * @brief
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
   * @brief Computes distance between nodes based on node indexes
   */
  using DistanceGetter = std::function<double(unsigned int, unsigned int)>;

  SearchTree() = default;

  ~SearchTree() {
    delete root_node_;
    delete target_node_;
  }

  inline void clear() {
    root_node_ = nullptr;
    target_node_ = nullptr;
    tree_.clear();
  }

  inline void reserve(unsigned int size) { tree_.reserve(size); }

  inline void addVertex(const NodePtr& node) { tree_.push_back(node); }

  inline bool isNodeInTree(const NodePtr& node) {
    return std::find(tree_.begin(), tree_.end(), node) != tree_.end();
  }

  inline void setRootNode(const NodePtr& node) {
    root_node_ = node;
    addVertex(node);
  }

  inline void setTargetNode(const NodePtr& node) { target_node_ = node; }

  /**
   * @brief
   * @param index
   * @param distance_getter
   * @return NodePtr
   */
  NodePtr getClosestNode(unsigned int index,
                         DistanceGetter& distance_getter) const {
    if (tree_.empty()) {
      return nullptr;
    }

    auto nearest_it =
        std::min_element(tree_.cbegin(), tree_.cend(),
                         [&](const auto& node1, const auto& node2) {
                           return distance_getter(node1->getIndex(), index) <
                                  distance_getter(node2->getIndex(), index);
                         });
    // TODO: Optimize this later to be one statement
    return (*nearest_it);
  }

  /**
   * @brief
   * @param index
   * @param near_nodes
   * @param distance_getter
   * @param near_distance
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
   * @brief
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
