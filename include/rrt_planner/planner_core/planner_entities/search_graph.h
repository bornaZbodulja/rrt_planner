/**
 * @file search_graph.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Search graph implementation
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_GRAPH_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_GRAPH_H_

#include <unordered_map>

#include "rrt_planner/planner_core/planner_entities/node.h"

namespace rrt_planner::planner_core::planner_entities {
/**
 * @brief Holder for storing indexed nodes in the graph
 * @tparam NodeT Node template
 */
template <typename NodeT>
class SearchGraph {
 public:
  SearchGraph() = default;
  ~SearchGraph() = default;

  void reserve(std::size_t size) { graph_.reserve(size); }

  void clear() {
    // Resetting all nodes before deleting them!!!
    std::for_each(graph_.begin(), graph_.end(),
                  [&](auto& it) { it.second.reset(); });
    graph_.clear();
  }

  /**
   * @brief Gets node from graph at given index
   * @param index Given index
   * @return NodeT* Pointer to node at given index
   */
  NodeT* getNode(unsigned int index) {
    auto it = graph_.find(index);

    if (it == graph_.end()) {
      // if node with given index doesn't exist in graph, create it
      return &(graph_.emplace(index, NodeT(index)).first->second);
    }

    return &(it->second);
  }

 private:
  std::unordered_map<unsigned int, NodeT> graph_;
};  // namespace rrt_planner::planner_core
}  // namespace rrt_planner::planner_core::planner_entities

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_GRAPH_H_
