/**
 * @file tree_connector.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Tree connector interface
 * @version 0.1
 * @date 2024-02-18
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTOR_H_
#define RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTOR_H_

#include <memory>
#include <optional>
#include <vector>

#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"

namespace rrt_planner::planner_core::tree_connector {
template <typename StateT>
class TreeConnector {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

  virtual ~TreeConnector() = default;

  /**
   * @brief Tries to connect given node from one search tree to second search
   * tree
   * @param node Given node from first search tree
   * @param tree Pointer to another search tree
   * @return ConnectionResultT If connection is successful, returns pointer to
   * closest node in second tree, otherwise returns null pointer
   */
  virtual NodeT* tryConnectTrees(
      const NodeT* node,
      const rrt_planner::planner_core::planner_entities::SearchTree<
          StateT>* const tree) = 0;

 protected:
  TreeConnector() = default;
};
}  // namespace rrt_planner::planner_core::tree_connector

#endif  // RRT_PLANNER__PLANNER_CORE__TREE_CONNECTOR__TREE_CONNECTOR_H_
