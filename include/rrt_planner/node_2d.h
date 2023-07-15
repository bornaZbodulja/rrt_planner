/**
 * @file node_2d.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Node 2D implementation
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
#include "rrt_planner/search_info.h"

namespace rrt_planner {

/**
 * @brief Holder for all relevant motion params
 */
struct MotionTable2D {
  /**
   * @brief Constructor for 2D motion table
   */
  MotionTable2D(){};

  /**
   * @brief Initializes motion table
   * @param size_x_in Width of costmap
   * @param search_info Planner search info
   */
  void Initialize(const unsigned int size_x_in, const SearchInfo& search_info);

  // Motion model
  MotionModel motion_model{MotionModel::UNKNOWN};
  // Width of costmap
  unsigned int size_x{0};
  // Cost penalty
  double cell_cost_multiplier{0.0};
};

/**
 * @brief Node2D class implementation
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
    Coordinates(const double& x_in, const double& y_in) : x(x_in), y(y_in) {}

    double x{0}, y{0};
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
   * @brief Gets costmap cost reference
   * @return double
   */
  inline double GetCost() { return cell_cost_; }

  /**
   * @brief Sets costmap cost reference
   * @param cost Costmap cost
   */
  inline void SetCost(const double& cost) { cell_cost_ = cost; }

  /**
   * @brief Gets accumulated cost to approach this node
   * @return double
   */
  inline double GetAccumulatedCost() { return accumulated_cost_; }

  /**
   * @brief Sets accumulated cost to approach this node
   * @param cost
   */
  inline void SetAccumulatedCost(const double& cost) {
    accumulated_cost_ = cost;
  }

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
   * @brief Tries to extend this node towards given coordinates
   * @param coordinates Given coordinates for expansion
   * @param collision_checker Collision checker pointer
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow unknown costs
   * @param edge_length Length of edge in search tree
   * @return Index of extended node if connection is valid, nullopt otherwise
   */
  std::optional<unsigned int> ExtendNode(
      const Coordinates& coordinates,
      const CollisionCheckerPtr& collision_checker,
      const unsigned char& lethal_cost, const bool& allow_unknown,
      const int& edge_length = std::numeric_limits<int>::max());

  /**
   * @brief Rewires this node
   * @param parent New parent node pointer
   * @param accumulated_cost New accumulated cost
   */
  void RewireNode(const NodePtr& parent, const double& accumulated_cost);

  /**
   * @brief Returns path connecting this and given node
   * @param node Node pointer
   * @return CoordinatesVector
   */
  CoordinatesVector ConnectNode(const NodePtr& node);

  /**
   * @brief Backtrace path to root node
   * @return NodeVector Path from this node to root
   */
  NodeVector BackTracePath();

  /**
   * @brief Initializes motion model for Node2D
   * @param size_x_in Width of costmap
   * @param search_info Planner search info
   */
  static void InitializeMotionTable(const unsigned int& size_x_in,
                                    const SearchInfo& search_info);

  /**
   * @brief Computes index based on coordinates
   * @param x X position in map frame
   * @param y Y position in map frame
   * @return unsigned int
   */
  static inline unsigned int GetIndex(const unsigned int& x,
                                      const unsigned int& y) {
    return y * motion_table.size_x + x;
  }

  /**
   * @brief Computes index based on coordinates
   * @param coordinates
   * @return unsigned int
   */
  static inline unsigned int GetIndex(const Coordinates& coordinates) {
    return GetIndex(static_cast<unsigned int>(coordinates.x),
                    static_cast<unsigned int>(coordinates.y));
  }

  /**
   * @brief Generates coordinates from map cell index
   * @param index Map cell index
   * @return Coordinates
   */
  static inline Coordinates GetCoordinates(const unsigned int& index) {
    return Coordinates(index % motion_table.size_x,
                       index / motion_table.size_x);
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

  // Motion table
  inline static MotionTable2D motion_table{};

  // Parent node
  NodePtr parent;
  // Map cell coordinates of the node
  Coordinates coordinates;

 private:
  // Map cell index of the node
  unsigned int index_;
  // Whether node was visited
  bool visited_;
  // Cost of the map cell associated with node
  double cell_cost_;
  // Accumulated cost of the node
  double accumulated_cost_;
};

}  // namespace rrt_planner

#endif
