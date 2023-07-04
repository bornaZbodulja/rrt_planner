/**
 * @file node_hybrid.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Definition of Node Hybrid
 * @version 0.1
 * @date 2023-05-28
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__NODE_HYBRID_H_
#define RRT_PLANNER__NODE_HYBRID_H_

#include <ompl/base/StateSpace.h>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "rrt_planner/analytic_expansion.h"
#include "rrt_planner/constants.h"
#include "rrt_planner/types.h"

namespace rrt_planner {

// Forward declare analytic expansion
template <typename NodeT>
class AnalyticExpansion;

/**
 * @brief Holder for all relevant motion params
 */
struct HybridMotionTable {
  /**
   * @brief Constructor for hybrid motion table
   */
  HybridMotionTable() {}

  /**
   * @brief Initializes motion table
   * @param size_x_in Width of costmap
   * @param angle_bin_size_in
   * @param search_info Planner search info
   * @param motion_model_in Motion model
   */
  void Initialize(const unsigned int& size_x_in,
                  const unsigned int& angle_bin_size_in,
                  const SearchInfo& search_info,
                  const MotionModel& motion_model_in);

  /**
   * @brief Gets angular bin
   * @param theta Raw orientation
   * @return int Index of bin
   */
  inline int GetClosestAngularBin(const double& theta) const {
    return static_cast<int>(std::floor(theta / angle_bin));
  }

  /**
   * @brief Gets raw orientation from bin
   * @param bin_idx Bin index
   * @return double Orientation of bin
   */
  inline double GetAngleFromBin(const int& bin_idx) const {
    return bin_idx * angle_bin;
  }

  // Motion model (DUBINS or REEDS-SHEPP)
  MotionModel motion_model{MotionModel::UNKNOWN};
  // State space pointer (DUBINS or REEDS-SHEPP)
  ompl::base::StateSpacePtr state_space;
  // Minimum turning radius
  double min_turning_radius{0.0};
  // Cost penalty
  double cell_cost_multiplier{0.0};
  // Width of costmap
  unsigned int size_x{0};
  // Angle bin size
  unsigned int angle_bin_size{0};
  // Angle bin
  double angle_bin{0.0};
};

/**
 * @brief NodeHybrid class implementation
 */
class NodeHybrid {
 public:
  typedef NodeHybrid* NodePtr;
  typedef std::vector<NodePtr> NodeVector;

  /**
   * @brief NodeHybrid implementation of coordinate structure
   */
  struct Coordinates {
    Coordinates() {}
    Coordinates(const double& x_in, const double& y_in, const double& theta_in)
        : x(x_in), y(y_in), theta(theta_in) {}

    double x{0}, y{0}, theta{0};
  };
  typedef std::vector<Coordinates> CoordinatesVector;

  /**
   * @brief Constructor for NodeHybrid
   * @param index Index of the node
   */
  explicit NodeHybrid(const unsigned int& index);

  /**
   * @brief Destructor for NodeHybrid
   */
  ~NodeHybrid();

  /**
   * @brief Comparison operator for NodeHybrid
   * @param rhs Right hand side node reference
   * @return True if indexes equal, false otherwise
   */
  bool operator==(const NodeHybrid& rhs) const {
    return index_ == rhs.GetIndex();
  }

  /**
   * @brief Reset method for new search
   */
  void Reset();

  /**
   * @brief Gets index of NodeHybrid
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
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow unknown costs
   * @return True if node is valid, false otherwise
   */
  bool IsNodeValid(const CollisionCheckerPtr& collision_checker,
                   const unsigned char& lethal_cost, const bool& allow_unknown);

  /**
   * @brief Tries to extend this node towards given coordinates
   * @param coordinates Given coordinates for expansion
   * @param collision_checker Collision checker pointer
   * @param lethal_cost Lethal cost for collision checker
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
   * @brief Returns path connecting this and given node
   * @param node Node pointer
   * @return CoordinatesVector
   */
  CoordinatesVector ConnectNode(const NodePtr& node);

  /**
   * @brief Rewires this node
   * @param parent New parent node pointer
   * @param accumulated_cost New accumulated cost
   */
  void RewireNode(const NodePtr& parent, const double& accumulated_cost);

  /**
   * @brief Backtrace path to root node
   * @return NodeVector Path from this node to root
   */
  NodeVector BackTracePath();

  /**
   * @brief Initializes motion model for NodeHybrid
   * @param size_x_in Width of costmap
   * @param angle_bin_size_in Number of angle bins
   * @param search_info Planner search info
   * @param motion_model Motion model
   */
  static void InitializeMotionModel(const unsigned int& size_x_in,
                                    const unsigned int& angle_bin_size_in,
                                    const SearchInfo& search_info,
                                    const MotionModel& motion_model);

  /**
   * @brief Computes index based on coordinates
   * @param x X position in map frame
   * @param y Y position in map frame
   * @param angle Index of angle bin
   * @return unsigned int
   */
  static inline unsigned int GetIndex(const unsigned int& x,
                                      const unsigned int& y,
                                      const unsigned int& angle) {
    return angle + x * motion_table.angle_bin_size +
           y * motion_table.size_x * motion_table.angle_bin_size;
  }

  /**
   * @brief Computes index based on coordinates
   * @param coordinates
   * @return unsigned int
   */
  static inline unsigned int GetIndex(const Coordinates& coordinates) {
    return GetIndex(static_cast<unsigned int>(coordinates.x),
                    static_cast<unsigned int>(coordinates.y),
                    static_cast<unsigned int>(coordinates.theta));
  }

  /**
   * @brief Generates coordinates from index
   * @param index Node index
   * @return Coordinates
   */
  static inline Coordinates GetCoordinates(const unsigned int& index) {
    return Coordinates(
        (index / motion_table.angle_bin_size) % motion_table.size_x,
        index / (motion_table.angle_bin_size * motion_table.size_x),
        index % motion_table.angle_bin_size);
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
                      first_coordinates.y - second_coordinates.y) +
           std::abs(second_coordinates.theta - first_coordinates.theta);
  }

  // Parent node
  NodePtr parent;
  // Coordinates of the node
  Coordinates coordinates;

  // Motion table
  inline static HybridMotionTable motion_table{};
  // Analytic expander
  inline static std::unique_ptr<AnalyticExpansion<NodeHybrid>> expander{};

 private:
  // Index of the node
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
