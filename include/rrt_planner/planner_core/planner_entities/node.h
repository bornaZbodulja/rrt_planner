/**
 * @file node.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Planner node definition
 * @version 0.1
 * @date 2023-09-16
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__NODE_H_
#define RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__NODE_H_

#include <limits>
#include <vector>

namespace rrt_planner::planner_core::planner_entities {
/**
 * @brief Node interface
 * @tparam T State template
 */
template <typename StateT>
class Node {
 public:
  explicit Node(const StateT& state)
      : parent(nullptr),
        state_(state),
        visited_(false),
        accumulated_cost_(std::numeric_limits<double>::max()) {}

  ~Node() { parent = nullptr; }

  const StateT& getState() const { return state_; }

  bool isVisited() const { return visited_; }

  double getAccumulatedCost() const { return accumulated_cost_; }

  void visited() { visited_ = true; }

  void setAccumulatedCost(double cost) { accumulated_cost_ = cost; }

  void reset() {
    parent = nullptr;
    visited_ = false;
    accumulated_cost_ = std::numeric_limits<double>::max();
  }

  bool operator==(const Node& rhs) const {
    return this->state_ == rhs.getState();
  }

  void rewireNode(const Node* new_parent, double accumulated_cost) {
    parent = new_parent;
    accumulated_cost_ = accumulated_cost;
  }

  /**
   * @brief Backtracks path to root node
   * @return std::vector<StateT>
   */
  std::vector<StateT> backTracePath() {
    std::vector<StateT> path;
    Node* current_node = this;

    while (current_node != nullptr) {
      path.push_back(current_node->getState());
      current_node = current_node->parent;
    }

    return path;
  };

  // Parent node
  Node* parent{nullptr};

 private:
  // State
  StateT state_;
  // Whether node was initialized
  bool visited_;
  // Accumulated cost of node
  double accumulated_cost_;
};
}  // namespace rrt_planner::planner_core::planner_entities

#endif  // RRT_PLANNER__PLANNER_CORE__PLANNER_ENTITIES__NODE_H_
