/**
 * @file nearest_neighbor_expander.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Nearest neighbor expander implementation
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_EXPANDER__NEAREST_NEIGHBOR_EXPANDER_H_
#define RRT_PLANNER__PLANNER_CORE__NEAREST_NEIGHBOR_EXPANDER__NEAREST_NEIGHBOR_EXPANDER_H_

#include <memory>

#include "rrt_planner/planner_core/cost_scorer/cost_scorer.h"
#include "rrt_planner/planner_core/expander/expander.h"
#include "rrt_planner/planner_core/planner_entities/search_graph.h"
#include "rrt_planner/planner_core/planner_entities/search_tree.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core::nearest_neighbor_expander {
/**
 * @brief Regular (basic) RRT planner expander
 * @tparam StateT
 */
template <typename StateT>
class NearestNeighborExpander
    : public rrt_planner::planner_core::expander::Expander<StateT> {
 public:
  using ExpanderT = rrt_planner::planner_core::expander::Expander<StateT>;
  using NodePtr = typename ExpanderT::NodePtr;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;
  using StateConnectorT = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = std::shared_ptr<StateConnectorT>;
  using SearchTreePtr = typename ExpanderT::SearchTreePtr;
  using SearchGraphPtr = typename ExpanderT::SearchGraphPtr;
  using CostScorerT =
      rrt_planner::planner_core::cost_scorer::CostScorer<StateT>;
  using CostScorerPtr = std::unique_ptr<CostScorerT>;

  /**
   * @brief Regular expander constructor
   * @param cost_scorer Cost scorer pointer
   * @param state_space State space pointer
   * @param state_connector State connector pointer
   */
  NearestNeighborExpander(CostScorerPtr&& cost_scorer,
                          const StateSpacePtr& state_space,
                          const StateConnectorPtr& state_connector)
      : ExpanderT(),
        cost_scorer_(std::move(cost_scorer)),
        state_space_(state_space),
        state_connector_(state_connector) {}

  ~NearestNeighborExpander() override = default;

  /**
   * @brief Tries to expand search tree based on given expansion index
   * @param expansion_index Expansion index used for generating new node for
   * expansion
   * @param tree Search tree pointer
   * @param graph Search graph pointer
   * @return NodePtr
   */
  NodePtr expandTree(unsigned int expansion_index, SearchTreePtr tree,
                     SearchGraphPtr graph) override {
    // Get closest node(state) to indexed state in state space
    NodePtr closest_node = tree->getClosestNode(expansion_index);
    // Get new node for expansion
    NodePtr new_node = getNewNode(expansion_index, closest_node, graph);

    if (new_node == nullptr) {
      return nullptr;
    }

    // If node is visited and not in tree, target is reached
    if (new_node->isVisited() && !tree->isNodeInTree(new_node)) {
      return new_node;
    }

    updateNode(new_node, closest_node, tree);

    return new_node;
  }

 protected:
  /**
   * @brief Generates new node for expansion based on expansion index
   * @param expansion_index Given expansion index
   * @param closest_node Closest node pointer
   * @param graph Search graph pointer
   * @return NodePtr
   */
  NodePtr getNewNode(unsigned int expansion_index, const NodePtr& closest_node,
                     SearchGraphPtr graph) {
    if (closest_node == nullptr) {
      return nullptr;
    }

    // Try extending closest node (state) towards expansion index (state)
    std::optional<StateT> new_state = state_connector_->expandState(
        closest_node->state, state_space_->getState(expansion_index));

    if (!new_state.has_value()) {
      return nullptr;
    }

    // Return node which corresponds to new state
    NodePtr new_node =
        graph->getNode(state_space_->getIndex(new_state.value()));
    new_node->state = new_state.value();
    return new_node;
  }

  /**
   * @brief Given node and parent node, backtracks through ancestors of parent
   * to find the last one which yields valid connection to new node as parent
   * and updates new node info accordingly
   * @param new_node New node to be updated
   * @param parent_node Parent node of new node
   * @param tree Search tree pointer
   */
  void updateNode(NodePtr new_node, NodePtr parent_node, SearchTreePtr tree) {
    // Backtrack through closest node ancestors
    parent_node = backTracking(new_node, parent_node);

    double accumulated_cost = computeAccumulatedCost(parent_node, new_node);
    bool new_node_visited = new_node->isVisited();
    bool should_update_node = accumulated_cost < new_node->getAccumulatedCost();

    if (should_update_node) {
      new_node->parent = parent_node;
      new_node->setAccumulatedCost(accumulated_cost);
      if (!new_node_visited) {
        new_node->visited();
        tree->addVertex(new_node);
      }
    }
  }

  /**
   * @brief Backtracks through ancestors of new node parent and chooses the last
   * one which yields valid connection to new node as parent
   * @param new_node Newly added node to search tree
   * @param parent_node Current parent node of new node
   * @return NodePtr
   */
  NodePtr backTracking(const NodePtr& new_node, const NodePtr& parent_node) {
    NodePtr current_anc = parent_node;

    while (current_anc->parent != nullptr) {
      if (state_connector_->tryConnectStates(current_anc->parent->state,
                                             new_node->state)) {
        current_anc = current_anc->parent;
      } else {
        break;
      }
    }

    return current_anc;
  }

  double computeAccumulatedCost(const NodePtr& parent_node,
                                const NodePtr& child_node) const {
    return parent_node->getAccumulatedCost() +
           getTraversingCost(parent_node->state, child_node->state);
  }

  double getTraversingCost(const StateT& parent_state,
                           const StateT& child_state) const {
    return (*cost_scorer_)(parent_state, child_state);
  }

  // Cost scorer pointer
  CostScorerPtr cost_scorer_;
  // State space pointer
  StateSpacePtr state_space_;
  // State connector pointer
  StateConnectorPtr state_connector_;
};
}  // namespace rrt_planner::planner_core::nearest_neighbor_expander

#endif
