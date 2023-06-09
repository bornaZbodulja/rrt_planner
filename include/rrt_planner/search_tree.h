/**
 * @file search_tree.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Search tree class implementation
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__SEARCH_TREE_H_
#define RRT_PLANNER__SEARCH_TREE_H_

#include <geometry_msgs/Pose.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "rrt_planner/node_2d.h"
#include "rrt_planner/node_hybrid.h"

namespace rrt_planner {

/**
 * @brief Search tree class implementation
 * @tparam NodeT
 */
template <typename NodeT>
class SearchTree {
 public:
  typedef NodeT* NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef typename NodeVector::iterator NodeIterator;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::pair<Coordinates, Coordinates> Edge;
  typedef std::vector<Edge> TreeMsg;

  /**
   * @brief Constructor for search tree
   */
  SearchTree() : root_node_(nullptr), target_node_(nullptr) {}

  /**
   * @brief Destructor for search tree
   */
  ~SearchTree();

  /**
   * @brief Reserves memory
   * @param size
   */
  inline void Reserve(const int& size) { tree_.reserve(size); }

  /**
   * @brief Clears memory
   */
  inline void Clear() { tree_.clear(); }

  /**
   * @brief Sets root node for the search tree
   * @param node Pointer to root node
   */
  inline void SetRootNode(const NodePtr& node) {
    root_node_ = node;
    AddVertex(root_node_);
  }

  /**
   * @brief Sets target node for the search tree
   * @param node Pointer to target node
   */
  inline void SetTargetNode(const NodePtr& node) { target_node_ = node; }

  /**
   * @brief Adds node to the search tree
   * @param node
   */
  inline void AddVertex(const NodePtr& node) { tree_.push_back(node); }

  /**
   * @brief Checks if given node in search tree
   * @param node
   * @return True if node in search tree, false otherwise
   */
  bool IsNodeInTree(const NodePtr& node);

  /**
   * @brief Gets closest node to the node associated with given index
   * @param index
   * @return NodePtr Closest node pointer
   */
  NodePtr GetClosestNode(const unsigned int& index);

  /**
   * @brief Gets all nodes which are in neighborhood of the associated with
   * given index
   * @param index
   * @param near_nodes Vector of near node pointers
   */
  void GetNearNodes(const unsigned int& index, NodeVector& near_nodes);

  /**
   * @brief
   * @param new_node Newly added node to tree
   * @param near_nodes Vector of near
   * @param collision_checker
   * @param lethal_cost
   * @param allow_unknown
   */
  void RewireTree(NodePtr& new_node, NodeVector& near_nodes,
                  const CollisionCheckerPtr& collision_checker,
                  const unsigned char& lethal_cost, const bool& allow_unknown);

  /**
   * @brief Generates msg form of search tree
   */
  TreeMsg TreeToMsg();

  // Defines neighborhood of the node
  inline static double near_distance{0.0};

 private:
  NodePtr root_node_, target_node_;
  NodeVector tree_;
};

}  // namespace rrt_planner

#endif
