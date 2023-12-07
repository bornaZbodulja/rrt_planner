/**
 * @file rrt_core.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT planner core interface
 * @version 0.1
 * @date 2023-09-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__RRT_CORE_H_
#define RRT_PLANNER__PLANNER_CORE__RRT_CORE_H_

#include <nav_utils/nav_utils.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "rrt_planner/planner_core/node.h"
#include "rrt_planner/planner_core/search_graph.h"
#include "rrt_planner/planner_core/search_params.h"
#include "rrt_planner/planner_core/search_tree.h"
#include "rrt_planner/planner_core/timeout_handler.h"
#include "state_space/state_connector/state_connector.h"
#include "state_space/state_sampler/state_sampler.h"
#include "state_space/state_space/state_space.h"

namespace rrt_planner::planner_core {
/**
 * @brief
 * @tparam StateT
 */
template <typename StateT>
class RRTCore {
 public:
  using NodeT = Node<StateT>;
  using NodePtr = NodeT*;
  using StateVector = std::vector<StateT>;
  using StateSpaceT = state_space::StateSpace<StateT>;
  using StateSpacePtr = std::shared_ptr<StateSpaceT>;
  using StateConnector = state_space::state_connector::StateConnector<StateT>;
  using StateConnectorPtr = std::unique_ptr<StateConnector>;
  using StateSamplerT = state_space::state_sampler::StateSampler<StateT>;
  using StateSamplerPtr = std::unique_ptr<StateSamplerT>;
  using SearchTreeT = SearchTree<NodeT>;
  using SearchTreePtr = std::unique_ptr<SearchTreeT>;
  using SearchGraphT = SearchGraph<NodeT>;
  using SearchGraphPtr = std::unique_ptr<SearchGraphT>;
  using SearchInfo = SearchParams;
  using DistanceGetterT = std::function<double(unsigned int, unsigned int)>;
  using PlanningResultT = std::optional<StateVector>;
  using EdgeT = std::pair<StateT, StateT>;
  using TreeMsgT = std::vector<EdgeT>;
  using TreeMsgVectorT = std::vector<TreeMsgT>;

  /**
   * @brief
   * @param state_space
   * @param state_connector
   * @param search_info
   * @param collision_checker
   */
  RRTCore(const StateSpacePtr& state_space, StateConnectorPtr&& state_connector,
          StateSamplerPtr&& state_sampler, SearchInfo&& search_info,
          const CollisionCheckerPtr& collision_checker)
      : start_tree_(std::make_unique<SearchTreeT>()),
        search_info_(std::move(search_info)),
        state_space_(state_space),
        state_connector_(std::move(state_connector)),
        collision_checker_(collision_checker),
        graph_(std::make_unique<SearchGraphT>()),
        state_sampler_(std::move(state_sampler)),
        timeout_handler_(TimeoutHandler(search_info_.max_planning_time)) {
    distance_getter_ = [&](unsigned int index1, unsigned int index2) -> double {
      return state_space_
          ->getStateDistance(state_space_->getState(index1),
                             state_space_->getState(index2))
          .norm();
    };

    reserveStartTree(search_info_.max_expansion_iterations);
    reserveGraph(search_info_.max_expansion_iterations);
  }

  virtual ~RRTCore() {
    clearStartTree();
    clearGraph();
    delete start_;
    delete goal_;
  };

  /**
   * @brief Initializes search by clearing memory for search graph and tree
   */
  virtual void initializeSearch() {
    start_ = nullptr;
    goal_ = nullptr;
    clearStartTree();
    clearGraph();
    resetExpansionIterationsCounter();
  }

  /**
   * @brief Sets start node for search
   * @param start_state Start state
   */
  virtual void setStart(const StateT& start_state) {
    start_ = getNodeFromGraph(state_space_->getIndex(start_state));
    start_->state = start_state;
    start_->setAccumulatedCost(0.0);
    start_->setCellCost(
        collision_checker_->GetCost(static_cast<unsigned int>(start_state.x),
                                    static_cast<unsigned int>(start_state.y)));
    start_->visited();
    start_tree_->setRootNode(start_);
  }

  /**
   * @brief Sets goal node for search
   * @param goal_state Goal state
   */
  virtual void setGoal(const StateT& goal_state) {
    goal_ = getNodeFromGraph(state_space_->getIndex(goal_state));
    goal_->state = goal_state;
    goal_->setAccumulatedCost(0.0);
    goal_->setCellCost(
        collision_checker_->GetCost(static_cast<unsigned int>(goal_state.x),
                                    static_cast<unsigned int>(goal_state.y)));
    goal_->visited();
    start_tree_->setTargetNode(goal_);
  }

  /**
   * @brief Tries to plan a path from setted start to goal
   * @return PlanningResultT
   */
  virtual PlanningResultT createPath() = 0;

  virtual TreeMsgVectorT getSearchTrees() const {
    auto&& index_tree = start_tree_->getSearchTree();
    TreeMsgT state_tree;
    state_tree.reserve(index_tree.size());

    std::transform(index_tree.cbegin(), index_tree.cend(),
                   std::back_inserter(state_tree), [&](const auto& edge) {
                     return EdgeT{state_space_->getState(edge.first),
                                  state_space_->getState(edge.second)};
                   });

    return {state_tree};
  }

  inline void updateCollisionChecker(
      const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

 protected:
  /**
   * @brief Generates new index for search tree expansion
   * @param target_index Index of target node in search tree
   * @return unsigned int
   */
  inline unsigned int getNewExpansionIndex(unsigned int target_index) {
    return state_sampler_->generateTreeExpansionIndex(
        target_index, state_space_.get(), collision_checker_);
  }

  /**
   * @brief
   * @param index
   * @param search_tree
   * @return NodePtr
   */
  inline NodePtr getClosestNode(unsigned int index,
                                const SearchTreePtr& search_tree) {
    return search_tree->getClosestNode(index, distance_getter_);
  }

  /**
   * @brief Backtracks through ancestors of new node parent and chooses the last
   * one which yields valid connection to new node as parent
   * @param new_node Newly added node to search tree
   * @param parent_node Current parent node of new node
   * @return NodePtr
   */
  NodePtr backTracking(NodePtr& new_node, NodePtr& parent_node) {
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

  /**
   * @brief Computes costmap cost of the node
   * @param node
   * @return double
   */
  inline double getCostmapCost(const NodePtr& node) {
    return search_info_.cost_penalty * node->getCellCost();
  }

  /**
   * @brief Computes cost of traversing from parent to child state
   * @param parent_state
   * @param child_state
   * @return double
   */
  inline double getTraversalCost(const NodePtr& parent_node,
                                 const NodePtr& child_node) const {
    return search_info_.traversal_cost *
           state_connector_->getStatesDistance(parent_node->state,
                                               child_node->state);
  }

  inline bool isGoal(const NodePtr& node) const {
    return node->getIndex() == goal_->getIndex();
  }

  inline unsigned int getStartIndex() const { return start_->getIndex(); }

  inline unsigned int getGoalIndex() const { return goal_->getIndex(); }

  inline void addNodeToTree(const NodePtr& node, SearchTreePtr& tree) {
    tree->addVertex(node);
  }

  inline NodePtr getNodeFromGraph(unsigned int index) {
    return graph_->getNode(index);
  }

  inline void clearGraph() { graph_->clear(); }

  inline void reserveGraph(unsigned int size) { graph_->reserve(size); }

  inline void clearStartTree() { start_tree_->clear(); }

  inline void reserveStartTree(unsigned int size) {
    start_tree_->reserve(size);
  }

  inline void setPlanningStartTime() { timeout_handler_.setStartTime(); }

  inline double getElapsedTime() const {
    return timeout_handler_.getElapsedTime();
  }

  inline bool isPlanningTimeoutReached() const {
    return timeout_handler_.timeoutReached();
  }

  inline void resetExpansionIterationsCounter() { iterations_counter_ = 0; }

  inline int getCurrentExpansionIterationCounter() {
    return iterations_counter_;
  }

  inline unsigned int getMaximumNumberOfIterations() {
    return search_info_.max_expansion_iterations;
  }

  inline bool reachedMaximumNumberOfIterations() {
    return ++iterations_counter_ > search_info_.max_expansion_iterations;
  }

  inline bool planningExpired() {
    return isPlanningTimeoutReached() || reachedMaximumNumberOfIterations();
  }

  // Start node pointer
  NodePtr start_{nullptr};
  // Goal node pointer
  NodePtr goal_{nullptr};
  // Search tree with start as root node
  SearchTreePtr start_tree_;
  // Planning search info
  SearchInfo search_info_;
  // State space pointer
  StateSpacePtr state_space_;
  // State connector pointer
  StateConnectorPtr state_connector_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;

 private:
  // Search graph
  SearchGraphPtr graph_;
  // State sampler
  StateSamplerPtr state_sampler_;
  // Distance getter between states
  DistanceGetterT distance_getter_;
  // Planning timeout handler
  TimeoutHandler timeout_handler_;
  // Planning expansions iterations counter
  int iterations_counter_{0};
};

}  // namespace rrt_planner::planner_core

#endif
