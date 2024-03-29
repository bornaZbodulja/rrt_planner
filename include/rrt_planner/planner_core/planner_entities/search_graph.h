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

#include <algorithm>
#include <vector>

#include "rrt_planner/planner_core/planner_entities/node.h"

namespace rrt_planner::planner_core::planner_entities {
/**
 * @brief Holder for storing nodes in the graph
 * @tparam StateT State template
 */
template <typename StateT>
class SearchGraph {
 public:
  using NodeT = Node<StateT>;

  SearchGraph() = default;
  ~SearchGraph() = default;

  void reserve(std::size_t size) { graph_.reserve(size); }

  void clear() {
    // Resetting all nodes before deleting them!!!
    std::for_each(graph_.begin(), graph_.end(),
                  [&](NodeT& node) { node.reset(); });
    graph_.clear();
  }

  /**
   * @brief Gets node from graph at given state
   * @param state Given state
   * @return NodeT* Pointer to node at given state
   */
  NodeT* getNode(const StateT& state) {
    auto it = std::find_if(
        graph_.begin(), graph_.end(),
        [&state](const NodeT& node) { return node.getState() == state; });

    if (it == graph_.end()) {
      // if node with given state doesn't exist in graph, create it
      return &(*(graph_.emplace(graph_.end(), state)));
    }

    return &(*it);
  }

 private:
  std::vector<NodeT> graph_;
};
}  // namespace rrt_planner::planner_core::planner_entities

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__SEARCH_GRAPH_H_
