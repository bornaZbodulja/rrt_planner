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
#include <memory>
#include <vector>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/expander_utilities/cost_scorer_utilities.h"
#include "rrt_planner/planner_core/nearest_neighbor_expander/nearest_neighbor_expander.h"
#include "rrt_planner/planner_core/nearest_neighbor_star_expander/nearest_neighbor_star_expander_params.h"
#include "rrt_planner/planner_core/planner_entities/node.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "state_space/state_connector/state_connector.h"

namespace rrt_planner::planner_core::nearest_neighbor_star_expander {
/**
 * @brief RRT* planner expander
 * @tparam StateT
 */
template <typename StateT>
class NearestNeighborStarExpander
    : public rrt_planner::planner_core::expander::Expander<StateT> {
 public:
  using NodeT = rrt_planner::planner_core::planner_entities::Node<StateT>;

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
      : cost_scorer_{std::move(cost_scorer)},
        star_expander_params_{std::move(star_expander_params)},
        state_connector_(state_connector) {
    // Create deep copy of cost scorer
    std::unique_ptr<rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>
        cost_scorer_copy{
            new rrt_planner::planner_core::cost_scorer::CostScorer<StateT>{
                *cost_scorer_}};
    nearest_neighbor_expander_ =
        std::make_unique<rrt_planner::planner_core::nearest_neighbor_expander::
                             NearestNeighborExpander<StateT>>(
            std::move(cost_scorer_copy), state_connector_);
  }

  ~NearestNeighborStarExpander() override = default;

  /**
   * @brief
   * @param expansion_state Given expansion state
   * @param tree Search tree pointer
   * @return NodeT*
   */
  NodeT* expandTree(
      const StateT& expansion_state,
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>* const
          tree) override {
    // Get new node for expansion
    NodeT* new_node =
        nearest_neighbor_expander_->expandTree(expansion_state, tree);

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
          rrt_planner::planner_core::expander_utilities::
              computeAccumulatedCostCost(parent_node, new_node,
                                         cost_scorer_.get()));
    }

    // If rewire_tree flag is set, rewire nodes in near nodes vector
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
          new_approach_cost = rrt_planner::planner_core::expander_utilities::
              computeAccumulatedCostCost(potential_parent, child_node,
                                         cost_scorer_.get());
          if (new_approach_cost < current_cost &&
              state_connector_->tryConnectStates(potential_parent->getState(),
                                                 child_node->getState())) {
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
          new_approach_cost = rrt_planner::planner_core::expander_utilities::
              computeAccumulatedCostCost(potential_parent, potential_child,
                                         cost_scorer_.get());
          if (new_approach_cost < potential_child->getAccumulatedCost() &&
              state_connector_->tryConnectStates(potential_parent->getState(),
                                                 potential_child->getState())) {
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
      rrt_planner::planner_core::planner_entities::SearchTree<StateT>* const
          tree) const {
    return tree->getNearNodes(state, star_expander_params_.near_distance);
  }

  // Cost scorer pointer
  std::unique_ptr<rrt_planner::planner_core::cost_scorer::CostScorer<StateT>>
      cost_scorer_;
  // Expander parameters
  NearestNeighborStarExpanderParams star_expander_params_;
  // State connector pointer
  std::shared_ptr<state_space::state_connector::StateConnector<StateT>>
      state_connector_;
  // Nearest neighbor expander
  std::unique_ptr<rrt_planner::planner_core::nearest_neighbor_expander::
                      NearestNeighborExpander<StateT>>
      nearest_neighbor_expander_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_star_expander

#endif  // RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_STAR_EXPANDER__NEAREST_NEIGHBOR_STAR_EXPANDER_H_
