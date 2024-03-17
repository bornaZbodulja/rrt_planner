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

namespace rrt_planner::planner_core::planner_entities {
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
  using EdgeVectorT = std::vector<EdgeT>;

  /**
   * @brief Computes distance between states in state space associated with
   * given indexes
   */
  using DistanceGetter = std::function<double(unsigned int, unsigned int)>;

  SearchTree(DistanceGetter&& distance_getter)
      : distance_getter_(std::move(distance_getter)) {}

  ~SearchTree() = default;

  void clear() {
    root_node_ = nullptr;
    target_node_ = nullptr;
    tree_.clear();
  }

  void reserve(std::size_t size) { tree_.reserve(size); }

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
   * @return NodePtr Pointer to closest node
   */
  NodePtr getClosestNode(unsigned int index) const {
    if (tree_.empty()) {
      return nullptr;
    }

    return *std::min_element(
        tree_.cbegin(), tree_.cend(),
        [this, index](const NodePtr& node1, const NodePtr& node2) {
          return distance_getter_(node1->getIndex(), index) <
                 distance_getter_(node2->getIndex(), index);
        });
  }

  /**
   * @brief Returns vector of nodes in neighborhood of state associated with
   * given index
   * @param index Given index
   * @param near_distance Defines neighborhood of nodes
   */
  NodeVector getNearNodes(unsigned int index, double near_distance) const {
    NodeVector near_nodes{};

    if (tree_.empty()) {
      return {};
    }

    std::copy_if(tree_.cbegin(), tree_.cend(), std::back_inserter(near_nodes),
                 [&](const NodePtr& node) {
                   return distance_getter_(node->getIndex(), index) <=
                          near_distance;
                 });

    return near_nodes;
  }

  /**
   * @brief Returns search tree as vector of pairs of indexes where each pair
   * represents parent-child relation in search tree
   * @return TreeT
   */
  EdgeVectorT getTreeEdges() const {
    EdgeVectorT edges;
    edges.reserve(tree_.size());

    std::for_each(tree_.cbegin(), tree_.cend(), [&](const NodePtr& node) {
      if (node != nullptr && node->parent != nullptr) {
        edges.emplace_back(node->parent->getIndex(), node->getIndex());
      }
    });

    return edges;
  }

 private:
  // Root node pointer
  NodePtr root_node_{nullptr};
  // Target node pointer
  NodePtr target_node_{nullptr};
  // Holder for nodes in tree
  NodeVector tree_;
  // Distance getter
  DistanceGetter distance_getter_;
};

}  // namespace rrt_planner::planner_core::planner_entities

#endif
