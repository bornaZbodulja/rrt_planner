/**
 * @file analytic_expansion.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Analytic expansion implementation
 * @version 0.1
 * @date 2023-06-01
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__ANALYTIC_EXPANSION_H_
#define RRT_PLANNER__ANALYTIC_EXPANSION_H_

#include <ompl/base/ScopedState.h>

#include <cmath>
#include <optional>
#include <vector>

#include "nav_utils/nav_utils.h"
#include "rrt_planner/motion_model.h"
#include "rrt_planner/node_2d.h"
#include "rrt_planner/node_hybrid.h"

namespace rrt_planner {

/**
 * @brief Analytic expansion class implementation
 * @tparam NodeT
 */
template <typename NodeT>
class AnalyticExpansion {
 public:
  typedef NodeT* NodePtr;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::vector<Coordinates> CoordinatesVector;
  typedef std::optional<Coordinates> ExpansionResult;

  /**
   * @brief Empty constructor for analytic expander
   */
  AnalyticExpansion() {}

  /**
   * @brief Updates motion model for the expander
   * @param motion_model Motion model
   */
  inline void UpdateMotionModel(const MotionModel& motion_model) {
    motion_model_ = motion_model;
  }

  /**
   * @brief Updates collision checker for the expander
   * @param collision_checker Collision checker pointer
   */
  inline void UpdateCollisionChecker(
      const CollisionCheckerPtr& collision_checker) {
    collision_checker_ = collision_checker;
  }

  /**
   * @brief
   * @param start Start coordinates
   * @param goal Goal coordinates
   * @param node
   * @param lethal_cost Lethal cost for collision checking
   * @param allow_unknown Whether to allow unknown costs
   * @param max_length Max length of analytic path (in map cells)
   * @return ExpansionResult
   */
  ExpansionResult TryAnalyticExpansion(
      const Coordinates& start, const Coordinates& goal, const NodePtr& node,
      const unsigned char& lethal_cost, const bool& allow_unknown,
      const int& max_length = std::numeric_limits<int>::max()) const;

  /**
   * @brief Gets analytic path connecting two poses
   * @param start Start coordinates
   * @param goal Goal coordinates
   * @param node Node pointer
   * @return CoordinatesVector Analytic path connecting two poses
   */
  CoordinatesVector GetAnalyticPath(const Coordinates& start,
                                    const Coordinates& goal,
                                    const NodePtr& node) const;

  /**
   * @brief Gets length of analytic path connecting two poses
   * @param start Start coordinates
   * @param goal Goal coordinates
   * @param node Node pointer
   * @return double Length of analytic path connecting start and goal
   */
  double GetAnalyticPathLength(const Coordinates& start,
                               const Coordinates& goal,
                               const NodePtr& node) const;

 private:
  /**
   * @brief Populates state space from coordinates
   * @param state Scoped state
   * @param coordinates
   * @param node Node pointer
   */
  void CoordinatesToStateSpace(ompl::base::ScopedState<>& state,
                               const Coordinates& coordinates,
                               const NodePtr& node) const;

  // Motion model
  MotionModel motion_model_{MotionModel::UNKNOWN};
  // Collision checker pointer
  CollisionCheckerPtr collision_checker_;
};

}  // namespace rrt_planner

#endif
