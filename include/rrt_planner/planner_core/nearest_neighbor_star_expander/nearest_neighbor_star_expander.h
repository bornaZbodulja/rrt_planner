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
  using NearestNeighborExpanderT = rrt_planner::planner_core::
      nearest_neighbor_expander::NearestNeighborExpander<StateT>;
  using NodePtr = typename NearestNeighborExpanderT::NodePtr;
  using NodeVector = std::vector<NodePtr>;
  using StateSpacePtr = typename NearestNeighborExpanderT::StateSpacePtr;
  using StateConnectorPtr =
      typename NearestNeighborExpanderT::StateConnectorPtr;
  using SearchTreePtr = typename NearestNeighborExpanderT::SearchTreePtr;
  using SearchGraphPtr = typename NearestNeighborExpanderT::SearchGraphPtr;
  using CostScorerT =
      rrt_planner::planner_core::cost_scorer::CostScorer<StateT>;
  using CostScorerPtr = std::unique_ptr<CostScorerT>;

  /**
   * @brief Star expander constructor
   * @param star_expander_params Expander parameters
   * @param cost_scorer Cost scorer pointer
   * @param state_space State space pointer
   * @param state_connector State connector pointer
   */
  NearestNeighborStarExpander(
      NearestNeighborStarExpanderParams&& star_expander_params,
      CostScorerPtr&& cost_scorer, const StateSpacePtr& state_space,
      const StateConnectorPtr& state_connector)
      : NearestNeighborExpanderT(std::move(cost_scorer), state_space,
                                 state_connector),
        star_expander_params_(std::move(star_expander_params)) {}

  ~NearestNeighborStarExpander() override = default;

  /**
   * @brief
   * @param expansion_index Given expansion index
   * @param tree Search tree pointer
   * @param graph Search graph pointer
   * @return NodePtr
   */
  NodePtr expandTree(unsigned int expansion_index, SearchTreePtr tree,
                     SearchGraphPtr graph) override {
    // Get closest node(state) to indexed state in state space
    NodePtr closest_node = tree->getClosestNode(expansion_index);
    // Get new node for expansion
    NodePtr new_node = NearestNeighborExpanderT::getNewNode(
        expansion_index, closest_node, graph);

    if (new_node == nullptr) {
      return nullptr;
    }

    // If node is visited and not in tree, target is reached
    if (new_node->isVisited() && !tree->isNodeInTree(new_node)) {
      return new_node;
    }

    // Get near nodes for new node and select parent node for new node among
    // them
    NodeVector near_nodes = getNearNodes(new_node->getIndex(), tree);
    NodePtr parent_node = selectBestParent(new_node, near_nodes);

    // If parent node wasn't found among near nodes, use closest node as parent
    // node
    if (parent_node == nullptr) {
      parent_node = closest_node;
    }

    NearestNeighborExpanderT::updateNode(new_node, parent_node, tree);

    if (star_expander_params_.rewire_tree) {
      rewireNodes(new_node, near_nodes);
    }

    return new_node;
  }

 protected:
  /**
   * @brief Chooses parent which yields lowest accumulated cost towards child
   * node
   * @param child_node Child node
   * @param potential_parents Vector of potential parents
   * @return NodePtr
   */
  virtual NodePtr selectBestParent(const NodePtr& child_node,
                                   const NodeVector& potential_parents) {
    if (child_node == nullptr || potential_parents.empty()) {
      return nullptr;
    }

    NodePtr best_parent{nullptr};
    double min_cost{std::numeric_limits<double>::max()};
    double current_cost{0.0};

    std::for_each(potential_parents.cbegin(), potential_parents.cend(),
                  [&](const NodePtr& potential_parent) {
                    current_cost =
                        NearestNeighborExpanderT::computeAccumulatedCost(
                            potential_parent, child_node);
                    if (current_cost < min_cost &&
                        this->state_connector_->tryConnectStates(
                            potential_parent->state, child_node->state)) {
                      min_cost = current_cost;
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
  virtual void rewireNodes(const NodePtr& potential_parent,
                           NodeVector& potential_children) {
    if (potential_parent == nullptr || potential_children.empty()) {
      return;
    }

    double new_approach_cost{0.0};

    std::for_each(
        potential_children.begin(), potential_children.end(),
        [&](NodePtr potential_child) {
          new_approach_cost = NearestNeighborExpanderT::computeAccumulatedCost(
              potential_parent, potential_child);
          if (new_approach_cost < potential_child->getAccumulatedCost() &&
              this->state_connector_->tryConnectStates(
                  potential_parent->state, potential_child->state)) {
            potential_child->parent = potential_parent;
            potential_child->setAccumulatedCost(new_approach_cost);
          }
        });
  }

  /**
   * @brief Gets nodes in neighborhood of given index(state) in given search
   * tree
   * @param index Index(state) for which near nodes are to be found
   * @param tree Search tree pointer
   * @return NodeVector Vector of near nodes for given index in given search
   * tree
   */
  NodeVector getNearNodes(unsigned int index, const SearchTreePtr& tree) const {
    return tree->getNearNodes(index, star_expander_params_.near_distance);
  }

  // Expander parameters
  NearestNeighborStarExpanderParams star_expander_params_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_star_expander

#endif
