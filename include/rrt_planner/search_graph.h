/**
 * @file search_graph.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Search graph definition
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__SEARCH_GRAPH_H_
#define RRT_PLANNER__SEARCH_GRAPH_H_

#include <unordered_map>

namespace rrt_planner {

/**
 * @brief Holder for storing indexed nodes in the graph
 */
template <typename NodeT>
class SearchGraph {
 public:
  typedef NodeT *NodePtr;
  typedef std::unordered_map<unsigned int, NodeT> NodeGraph;

  /**
   * @brief Default empty constructor
   */
  SearchGraph() = default;

  /**
   * @brief Reserves memory
   * @param size
   */
  void Reserve(const int &size) { graph_.reserve(size); }

  /**
   * @brief Clears memory
   */
  void Clear() { graph_.clear(); }

  /**
   * @brief Gets node at given index
   * @param index
   * @return Pointer to node at given index
   */
  NodePtr GetNode(const unsigned int &index) {
    auto it = graph_.find(index);

    if (it == graph_.end()) {
      // if node with given index doesn't exist in graph, creating it and
      // returning pointer
      return &(graph_.emplace(index, NodeT(index)).first->second);
    } else {
      return &(it->second);
    }
  }

  /**
   * @brief Adds node to graph
   * @param node
   * @return Pointer to added node in graph
   */
  NodePtr AddNode(const NodeT &node) {
    return &(graph_.emplace(node.GetIndex(), node).first->second);
  }

 private:
  NodeGraph graph_;
};

}  // namespace rrt_planner

#endif
