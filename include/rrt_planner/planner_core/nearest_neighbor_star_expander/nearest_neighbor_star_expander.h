/**
 * @file nearest_neighbor_star_expander.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Nearest neighbor* expander implementation
 * @version 0.1
 * @date 2024-02-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_H_
#define RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_H_

#include <limits>
#include <vector>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/nearest_neighbor_expander/nearest_neighbor_expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander_params.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_graph.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "state_space/state_connector/state_connector.h"

namespace rrt_planner::planner_core::nearest_neighbor_star_expander {
/**
 * @brief RRT* planner expander
 * @tparam StateT
 */
template <typename StateT>
class NearestNeighborStarExpander
    : public rrt_planner::planner_core::nearest_neighbor_expander::
          NearestNeighborExpander<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;
  using NearestNeighborExpanderT = rrt_planner::planner_core::
      nearest_neighbor_expander::NearestNeighborExpander<StateT>;

  /**
   * @brief Star expander constructor
   * @param star_expander_params Expander parameters
   * @param cost_scorer Cost scorer pointer
   * @param state_connector State connector pointer
   */
  explicit NearestNeighborStarExpander(
      NearestNeighborStarExpanderParams&& star_expander_params,
      std::unique_ptr<
          rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>&&
          cost_scorer,
      const std::shared_ptr<
          state_space::state_connector::StateConnector<StateT>>&
          state_connector)
      : NearestNeighborExpanderT(std::move(cost_scorer), state_connector),
        star_expander_params_(std::move(star_expander_params)) {}

  ~NearestNeighborStarExpander() override = default;

  /**
   * @brief
   * @param expansion_state Given expansion state
   * @param tree Search tree pointer
   * @param graph Search graph pointer
   * @return NodeT*
   */
  NodeT* expandTree(
      const StateT& expansion_state,
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>* tree,
      rrt_planner::planner_core::planner_entities::SearchGraph<StateT>* graph)
      override {
    // Get new node for expansion
    NodeT* new_node =
        NearestNeighborExpanderT::expandTree(expansion_state, tree, graph);

    if (new_node == nullptr) {
      return nullptr;
    }

    // Get near nodes for new node and select parent node for new node among
    // them
    std::vector<NodeT*> near_nodes = getNearNodes(new_node->getState(), tree);
    NodeT* parent_node = selectBestParent(new_node, near_nodes);

    // If new parent node was found among near nodes, update parent and
    // accumulated cost for the node
    if (parent_node != nullptr) {
      new_node->parent = parent_node;
      new_node->setAccumulatedCost(
          NearestNeighborExpanderT::computeAccumulatedCost(parent_node,
                                                           new_node));
    }

    if (star_expander_params_.rewire_tree) {
      rewireNodes(new_node, near_nodes);
    }

    return new_node;
  }

 private:
  /**
   * @brief Selects parent which yields lowest accumulated cost towards child
   * node, if none of the parents can yield lower cost than current, nullptr is
   * returned
   * @param child_node Child node
   * @param potential_parents Vector of potential parents
   * @return NodeT*
   */
  NodeT* selectBestParent(const NodeT* child_node,
                          const std::vector<NodeT*>& potential_parents) {
    if (child_node == nullptr || potential_parents.empty()) {
      return nullptr;
    }

    NodeT* best_parent{nullptr};
    double new_approach_cost{std::numeric_limits<double>::max()};
    double current_cost{child_node->getAccumulatedCost()};

    std::for_each(
        potential_parents.begin(), potential_parents.end(),
        [&](NodeT* potential_parent) {
          new_approach_cost = NearestNeighborExpanderT::computeAccumulatedCost(
              potential_parent, child_node);
          if (new_approach_cost < current_cost &&
              this->state_connector_->tryConnectStates(
                  potential_parent->getState(), child_node->getState())) {
            current_cost = new_approach_cost;
            best_parent = potential_parent;
          }
        });

    return best_parent;
  }

  /**
   * @brief For each potential_children nodes checks whether lower cost approach
   * can be made using potential_parent node as parent, and if it can, rewires
   * node to new parent
   * @param potential_parent New potential parent
   * @param potential_children Vector of nodes to check if lower cost approach
   * can be made
   */
  void rewireNodes(NodeT* potential_parent,
                   std::vector<NodeT*>& potential_children) {
    if (potential_parent == nullptr || potential_children.empty()) {
      return;
    }

    double new_approach_cost{0.0};

    std::for_each(
        potential_children.begin(), potential_children.end(),
        [&](NodeT*& potential_child) {
          new_approach_cost = NearestNeighborExpanderT::computeAccumulatedCost(
              potential_parent, potential_child);
          if (new_approach_cost < potential_child->getAccumulatedCost() &&
              this->state_connector_->tryConnectStates(
                  potential_parent->getState(), potential_child->getState())) {
            potential_child->parent = potential_parent;
            potential_child->setAccumulatedCost(new_approach_cost);
          }
        });
  }

  /**
   * @brief Gets nodes in neighborhood of given state in given search
   * tree
   * @param state State for which near nodes are to be found
   * @param tree Search tree pointer
   * @return std::vector<NodeT> Vector of near nodes for given state in given
   * search tree
   */
  std::vector<NodeT*> getNearNodes(
      const StateT& state,
      const rrt_planner::planner_core::planner_entities::SearchTree<
          StateT>* const tree) const {
    return tree->getNearNodes(state, star_expander_params_.near_distance);
  }

  // Expander parameters
  NearestNeighborStarExpanderParams star_expander_params_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_star_expander

#endif  // RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_H_
