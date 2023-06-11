/**
 * @file rrt_star.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief RRT* algorithm definition
 * @version 0.1
 * @date 2023-05-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__RRT_STAR_H_
#define RRT_PLANNER__RRT_STAR_H_

#include <chrono>
#include <experimental/random>
#include <vector>

#include "nav_utils/nav_utils.h"
#include "rrt_planner/constants.h"
#include "rrt_planner/node_2d.h"
#include "rrt_planner/node_hybrid.h"
#include "rrt_planner/search_graph.h"
#include "rrt_planner/search_tree.h"
#include "rrt_planner/types.h"

namespace rrt_planner {

/**
 * @brief
 * @tparam NodeT
 */
template <typename NodeT>
class RRTStar {
 public:
  typedef NodeT* NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinatesVector CoordinatesVector;
  typedef std::pair<Coordinates, Coordinates> Edge;
  typedef std::vector<Edge> TreeMsg;

  /**
   * @brief Constructor for RRT*
   * @param motion_model Motion model (2D, Dubins, Reeds-Shepp)
   * @param search_info Search info
   * @param collision_checker Collision checker pointer
   */
  explicit RRTStar(const MotionModel& motion_model,
                   const SearchInfo& search_info,
                   const CollisionCheckerPtr& collision_checker);

  /**
   * @brief Destructor for RRT*
   */
  ~RRTStar();

  /**
   * @brief Initializes state space dimensions
   * @param size_x
   * @param size_y
   * @param dim_3
   */
  void InitializeStateSpace(const unsigned int& size_x,
                            const unsigned int& size_y,
                            const unsigned int& dim_3);

  /**
   * @brief Updates motion model for the planner
   * @param motion_model Motion model
   */
  inline void UpdateMotionModel(const MotionModel& motion_model) {
    motion_model_ = motion_model;
  }

  /**
   * @brief Updates search info for the planner
   * @param search_info Search info
   */
  inline void UpdateSearchInfo(const SearchInfo& search_info) {
    search_info_ = search_info;
  }

  /**
   * @brief Updates collision checker for the planner
   * @param collision_checker Collision checker pointer
   */
  inline void UpdateCollisionChecker(
      const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

  /**
   * @brief Sets the start for planning, as a node index
   * @param mx X coordinate in map frame
   * @param my Y coordinate in map frame
   * @param dim_3 Orientation bin
   */
  void SetStart(const unsigned int& mx, const unsigned int& my,
                const unsigned int& dim_3);

  /**
   * @brief Sets the goal for planning, as a node index
   * @param mx X coordinate in map frame
   * @param my Y coordinate in map frame
   * @param dim_3 Orientation bin
   */
  void SetGoal(const unsigned int& mx, const unsigned int& my,
               const unsigned int& dim_3);

  /**
   * @brief Gets reference to start tree
   * @return TreeMsg&
   */
  inline TreeMsg GetStartTree() { return start_tree_.TreeToMsg(); }

  /**
   * @brief Gets reference to goal tree
   * @return TreeMsg&
   */
  inline TreeMsg GetGoalTree() { return goal_tree_.TreeToMsg(); }

  /**
   * @brief Creates path from given start and goal
   * @param path Reference to vector of coordinates of generated path
   * @return True if path was successfully generated, false otherwise
   */
  bool CreatePath(CoordinatesVector& path);

 private:
  /**
   * @brief Initializes search graph and search trees
   * @param size Amount of memory to reserve for search graph
   * @param cost_penalty
   * @param near_distance
   */
  void InitializeSearch(const unsigned int& size, const double& near_distance);

  /**
   * @brief Extends search tree with passed index
   * @param index Generated index for tree expansion
   * @param tree Search tree
   * @param new_node Pointer to new node added to tree
   * @param closest_node Closest node to newly generated indexed one
   * @param near_nodes Neighbourhood of newly created node
   * @param edge_length Length of edges connecting nodes in search tree
   * @param rewire_tree Whether to rewire tree around newly added node
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow expansion in unknown areas
   * @return True if tree was successfully extended, false otherwise
   */
  bool ExtendTree(const unsigned int& index, SearchTree<NodeT>& tree,
                  NodePtr& new_node, NodePtr& closest_node,
                  NodeVector& near_nodes, const int& edge_length,
                  const bool& rewire_tree, const unsigned char& lethal_cost,
                  const bool& allow_unknown);

  /**
   * @brief Tries connecting second search tree with newly added node to first
   * search tree
   * @param new_node New node added to first search tree
   * @param closest_node Closest node to new node in second tree (filled by
   * method)
   * @param second_tree Second search tree
   * @param path Path connecting start and goal nodes
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow path to go to unknown areas
   * @return True if search trees are connected and path is created, false
   * otherwise
   */
  bool ConnectTrees(NodePtr& new_node, NodePtr& closest_node,
                    SearchTree<NodeT>& second_tree, CoordinatesVector& path,
                    const unsigned char& lethal_cost,
                    const bool& allow_unknown);

  /**
   * @brief Picks best parent for new node from near nodes
   * @param new_node
   * @param near_nodes
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow connection to go in unknown areas
   * @return NodePtr
   */
  NodePtr ChooseParent(NodePtr& new_node, NodeVector& near_nodes,
                       const unsigned char& lethal_cost,
                       const bool& allow_unknown);

  /**
   * @brief Prepares path by iterating through node vector and connecting nodes
   * @param path Node vector path from start to goal
   * @return CoordinatesVector
   */
  CoordinatesVector PreparePath(const NodeVector& path);

  /**
   * @brief Gets new index for tree expansion
   * @param target_bias
   * @param target_index
   * @param state_space_size
   * @return unsigned int
   */
  unsigned int GetNewIndex(const double& target_bias,
                           const unsigned int& target_index,
                           const unsigned int& state_space_size);

  /**
   * @brief Generates random index in state space
   * @param state_space_size State space size
   * @return unsigned int Random index
   */
  unsigned int GenerateRandomIndex(const unsigned int& state_space_size);

  /**
   * @brief Adds indexed node to search graph
   * @param index Index of the node
   * @return NodePtr Pointer to added node
   */
  inline NodePtr AddToGraph(const unsigned int& index) {
    return graph_.GetNode(index);
  }

  /**
   * @brief Clears search graph
   */
  inline void ClearGraph() { graph_.Clear(); }

  /**
   * @brief Reserves memory for graph
   */
  inline void ReserveGraph(const int& size) { graph_.Reserve(size); }

  // Start node pointer
  NodePtr start_;
  // Goal node pointer
  NodePtr goal_;
  // Search graph
  SearchGraph<NodeT> graph_;
  // First search tree
  SearchTree<NodeT> start_tree_;
  // Second search tree (doing bidirectional search)
  SearchTree<NodeT> goal_tree_;
  // Motion model
  MotionModel motion_model_;
  // Planning search info
  SearchInfo search_info_;
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
  // State space dimensions
  unsigned int size_x_, size_y_, dim_3_;
  // Utils for random number generation (TODO: move this to utils)
  std::minstd_rand gen;
  std::uniform_real_distribution<double> dist;
};

}  // namespace rrt_planner

#endif
