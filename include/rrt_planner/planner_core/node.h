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

#ifndef RRT_PLANNER__PLANNER_CORE__NODE_H_
#define RRT_PLANNER__PLANNER_CORE__NODE_H_

#include <limits>
#include <vector>

namespace rrt_planner::planner_core {
/**
 * @brief Node interface
 * @tparam T State template
 */
template <typename StateT>
class Node {
 public:
  using NodePtr = Node*;
  using NodeVector = std::vector<NodePtr>;
  using StateVector = std::vector<StateT>;

  Node(unsigned int index)
      : parent(nullptr),
        index_(index),
        visited_(false),
        cell_cost_(std::numeric_limits<double>::quiet_NaN()),
        accumulated_cost_(std::numeric_limits<double>::max()) {}

  ~Node() { parent = nullptr; }

  unsigned int getIndex() const { return index_; }

  bool isVisited() const { return visited_; }

  double getCellCost() const { return cell_cost_; }

  double getAccumulatedCost() const { return accumulated_cost_; }

  void visited() { visited_ = true; }

  void setCellCost(double cost) { cell_cost_ = cost; }

  void setAccumulatedCost(double cost) { accumulated_cost_ = cost; }

  void reset() {
    parent = nullptr;
    visited_ = false;
    cell_cost_ = std::numeric_limits<double>::quiet_NaN();
    accumulated_cost_ = std::numeric_limits<double>::max();
  }

  bool operator==(const Node& rhs) const { return index_ == rhs.getIndex(); }

  void rewireNode(const NodePtr& new_parent, double accumulated_cost) {
    parent = new_parent;
    accumulated_cost_ = accumulated_cost;
  }

  /**
   * @brief Backtracks path to root node
   * @return NodeVector
   */
  StateVector backTracePath() {
    StateVector path;
    auto current_node = this;

    while (current_node != nullptr) {
      path.push_back(current_node->state);
      current_node = current_node->parent;
    }

    return path;
  };

  // Parent node
  NodePtr parent{nullptr};
  // State
  StateT state;

 protected:
  // Index of node in state space
  unsigned int index_;
  // Whether node was initialized
  bool visited_;
  // Cost of the map cell associated with node
  double cell_cost_;
  // Accumulated cost of node
  double accumulated_cost_;
};

}  // namespace rrt_planner::planner_core

#endif
