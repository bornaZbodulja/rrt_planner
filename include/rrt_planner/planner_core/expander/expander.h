/**
 * @file expander.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Expander interface
 * @version 0.1
 * @date 2024-01-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__EXPANDER__EXPANDER_H_
#define RRT_PLANNER__PLANNER_CORE__EXPANDER__EXPANDER_H_

#include <memory>

#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_graph.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"

namespace rrt_planner::planner_core::expander {
/**
 * @brief Planner expander interface
 * @tparam StateT
 */
template <typename StateT>
class Expander {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

  virtual ~Expander() = default;

  /**
   * @brief
   * @param expansion_index
   * @param tree
   * @param graph
   * @return Node
   */
  virtual NodeT* expandTree(
      unsigned int expansion_index,
      rrt_planner::planner_core::planner_entities::SearchTree<NodeT>* tree,
      rrt_planner::planner_core::planner_entities::SearchGraph<NodeT>*
          graph) = 0;

 protected:
  Expander() = default;
};
}  // namespace rrt_planner::planner_core::expander

#endif  // RRT_PLANNER__PLANNER_CORE__EXPANDER__EXPANDER_H_
