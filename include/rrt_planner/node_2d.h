/**
 * @file node_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Definition of Node 2D
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__NODE_2D_H_
#define RRT_PLANNER__NODE_2D_H_

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "nav_utils/nav_utils.h"

namespace rrt_planner {

/**
 * @brief
 */
class Node2D {
 public:
  typedef Node2D* NodePtr;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @brief Node2D implementation of coordinate structure
   */
  struct Coordinates {
    Coordinates() {}
    Coordinates(const int& x_in, const int& y_in) : x(x_in), y(y_in) {}

    int x{0}, y{0};
  };
  typedef std::vector<Coordinates> CoordinatesVector;

  /**
   * @brief Constructor for Node2D
   * @param index The map cell index of the node
   */
  explicit Node2D(const unsigned int& index);

  /**
   * @brief Destructor for Node2D
   */
  ~Node2D();

  /**
   * @brief Comparison operator for Node2D
   * @param rhs Right hand side node reference
   * @return True if map cell indexes equal, false otherwise
   */
  bool operator==(const Node2D& rhs) const { return index_ == rhs.GetIndex(); }

  /**
   * @brief Reset method for new search
   */
  void Reset();

  /**
   * @brief Gets map cell index of Node2D
   * @return unsigned int
   */
  inline unsigned int GetIndex() const { return index_; }

  /**
   * @brief Whether node is already in search graph or not
   * @return True if node in search graph, false otherwise
   */
  inline bool IsVisited() const { return visited_; }

  /**
   * @brief Sets node as visited (node is in search graph)
   */
  inline void Visited() { visited_ = true; }

  /**
   * @brief Gets pointer to parent of the node
   * @return NodePtr
   */
  inline NodePtr GetParent() const { return parent_; }

  /**
   * @brief Sets pointer to parent of the node
   * @param parent
   */
  inline void SetParent(const NodePtr& parent) { parent_ = parent; }

  /**
   * @brief Gets costmap cost reference
   * @return double&
   */
  inline double GetCost() { return cell_cost_; }

  /**
   * @brief Sets costmap cost
   * @param cost Costmap cost
   */
  inline void SetCost(const double& cost) { cell_cost_ = cost; }

  /**
   * @brief Gets accumulated cost reference of this node
   * @return double&
   */
  inline double GetAccumulatedCost() { return accumulated_cost_; }

  /**
   * @brief Sets accumulated cost of this node
   * @param cost
   */
  inline void SetAccumulatedCost(const double& cost) {
    accumulated_cost_ = cost;
  }

  /**
   * @brief Gets coordinates of this node
   * @return Coordinates
   */
  inline Coordinates GetCoordinates() const { return coordinates_; }

  /**
   * @brief Computes traversal cost between this and child node
   * @param child Child node pointer
   * @return double Traversal cost
   */
  double GetTraversalCost(const NodePtr& child);

  /**
   * @brief Checks if node is valid
   * @param collision_checker Collision checker pointer
   * @param lethal_cost
   * @param allow_unknown Whether to allow unknown costs
   * @return True if node is valid, false otherwise
   */
  bool IsNodeValid(const CollisionCheckerPtr& collision_checker,
                   const unsigned char& lethal_cost, const bool& allow_unknown);

  /**
   * @brief Tries to connect this node with newly expanded one
   * @param index Given index for tree expansion
   * @param collision_checker Collision checker pointer
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow unknown costs
   * @param edge_length Length of edge in search tree
   * @return Index of connected node if connection is valid, nullopt otherwise
   */
  std::optional<unsigned int> ConnectNode(
      const unsigned int& index, const CollisionCheckerPtr& collision_checker,
      const unsigned char& lethal_cost, const bool& allow_unknown,
      const int& edge_length = std::numeric_limits<int>::max());

  /**
   * @brief Rewires this node
   * @param parent New parent node pointer
   * @param accumulated_cost New accumulated cost
   */
  void RewireNode(const NodePtr& parent, const double& accumulated_cost);

  /**
   * @brief Backtrace path to root node
   * @return Path from this node to root
   */
  CoordinatesVector BackTracePath();

  static inline unsigned int GetIndex(const unsigned int& x,
                                      const unsigned int& y) {
    return y * size_x + x;
  }

  /**
   * @brief Generate coordinates from map cell index
   * @param index Map cell index
   * @return Coordinates
   */
  static inline Coordinates GetCoordinates(const unsigned int& index) {
    const int m_y = index / size_x;
    const int m_x = index - (m_y * size_x);
    return Coordinates(m_x, m_y);
  }

  /**
   * @brief Computes distance between two coordinates
   * @param first_coordinates First coordinates
   * @param second_coordinates Second coordinates
   * @return double
   */
  static inline double CoordinatesDistance(
      const Coordinates& first_coordinates,
      const Coordinates& second_coordinates) {
    return std::hypot(first_coordinates.x - second_coordinates.x,
                      first_coordinates.y - second_coordinates.y);
  }

  // Cost penalty
  inline static double cost_travel_multiplier{0.0};
  // X size of the costmap
  inline static unsigned int size_x{0};

 private:
  // Map cell index of the node
  unsigned int index_;
  // Whether node was visited
  bool visited_;
  // Map cell coordinates if the node
  Coordinates coordinates_;
  // Parent node
  NodePtr parent_;
  // Cost of the map cell associated with node
  double cell_cost_;
  // Accumulated cost of the node
  double accumulated_cost_;
};

}  // namespace rrt_planner

#endif
