/**
 * @file entities_utilities.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Utility functions for working with planner entities
 * @version 0.1
 * @date 2024-03-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_UTILITIES__ENTITIES_UTILITIES_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_UTILITIES__ENTITIES_UTILITIES_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::planner_utilities {
/**
 * @brief Initializes search tree with distance getter
 * @param state_space State space pointer
 * @return Unique pointer to search tree
 */
template <typename StateT>
std::unique_ptr<rrt_planner::planner_core::planner_entities::SearchTree<StateT>>
createSearchTree(const state_space::StateSpace<StateT>* const state_space) {
  std::function<double(const StateT&, const StateT&)> distance_getter =
      [state_space](const StateT& state_a, const StateT& state_b) {
        return state_space->getStateDistance(state_a, state_b).squaredL2norm();
      };

  return std::make_unique<
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>>(
      std::move(distance_getter));
}

/**
 * @brief Transforms search tree to vector of state vectors representing edges
 * in search tree
 * @param tree Search tree pointer
 * @param state_space State space pointer
 * @param state_connector State connector pointer
 * @return std::vector<std::vector<StateT>>
 */
template <typename StateT>
std::vector<std::vector<StateT>> transformSearchTree(
    const rrt_planner::planner_core::planner_entities::SearchTree<StateT>* const
        tree,
    const state_space::StateSpace<StateT>* const state_space,
    const state_space::state_connector::StateConnector<StateT>* const
        state_connector) {
  std::vector<std::vector<StateT>> transformed_tree;
  std::vector<std::pair<StateT, StateT>> edges = tree->getTreeEdges();
  transformed_tree.reserve(edges.size());

  std::transform(
      edges.cbegin(), edges.cend(), std::back_inserter(transformed_tree),
      [state_space, state_connector](const std::pair<StateT, StateT>& edge) {
        return state_connector->connectStates(edge.first, edge.second);
      });
  return transformed_tree;
}

/**
 * @brief Creates a path by backtracking from given node to the root node
 * @tparam StateT
 * @param node Given node pointer
 * @param state_connector State connector pointer
 * @return std::vector<StateT> Interpolated path from given node to root node
 */
template <typename StateT>
std::vector<StateT> backtrackPathFromNodeToRoot(
    rrt_planner::planner_core::planner_entities::Node<StateT>* node,
    const state_space::state_connector::StateConnector<StateT>* const
        state_connector) {
  std::vector<StateT> raw_path = node->backTracePath();
  std::reverse(raw_path.begin(), raw_path.end());

  if (raw_path.size() < 2) {
    return raw_path;
  }

  std::vector<StateT> interpolated_path, path_segment;

  for (auto first_it = raw_path.begin(),
            second_it = std::next(raw_path.begin());
       second_it != raw_path.end(); first_it++, second_it++) {
    path_segment = state_connector->connectStates(*first_it, *second_it);
    std::move(path_segment.begin(), path_segment.end(),
              std::back_inserter(interpolated_path));
  }

  return interpolated_path;
}
}  // namespace rrt_planner::planner_core::planner_utilities

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_UTILITIES__ENTITIES_UTILITIES_H_
