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

#include <limits>
#include <vector>

namespace rrt_planner {

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
    Coordinates(const int& x_in, const int& y_in, const int& theta_in)
        : x(x_in), y(y_in), theta(theta_in) {}

    int x{0}, y{0}, theta{0};
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
   * @brief Gets coordinates of this node
   * @return Coordinates
   */
  inline Coordinates GetCoordinates() const { return coordinates_; }

  /**
   * @brief Rewires this node
   * @param parent New parent node pointer
   * @param accumulated_cost New accumulated cost
   */
  void RewireNode(const NodePtr& parent, const double& accumulated_cost);

  /**
   * @brief Backtrace path to root node
   * @return CoordinatesVector Path from this node to root
   */
  CoordinatesVector BackTracePath();

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
    return angle + x * angle_bin_size + y * size_x * angle_bin_size;
  }

  /**
   * @brief Generates coordinates from index
   * @param index Node index
   * @return Coordinates
   */
  static inline Coordinates GetCoordinates(const unsigned int& index) {
    return Coordinates((index / angle_bin_size) % size_x,
                       index / (angle_bin_size * size_x),
                       index % angle_bin_size);
  }

  // X size of costmap
  inline static unsigned int size_x{0};
  // Angle bin size
  inline static unsigned int angle_bin_size{0};

 private:
  // Index of the node
  unsigned int index_;
  // Whether node was visited
  bool visited_;
  // Coordinates of the node
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
