/**
 * @file visualization.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Visualization utility
 * @version 0.1
 * @date 2023-10-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__VISUALIZATION_H_

#include <ros/node_handle.h>

#include <memory>
#include <vector>

#include "rrt_planner/visualization_plugin/path_visualization.h"
#include "rrt_planner/visualization_plugin/search_tree_visualization.h"
#include "rrt_planner/visualization_plugin/search_tree_visualization_collection.h"

namespace rrt_planner::visualization {
class Visualization {
 public:
  using TreeVisualizationT = SearchTreeVisualizationCollection;
  using TreeVisualizationPtr = std::unique_ptr<TreeVisualizationT>;
  using PathVisualizationPtr = std::unique_ptr<PathVisualization>;
  using PlanT = PathVisualization::PlanT;
  using TreeT = SearchTreeVisualization::TreeT;
  using TreeVector = std::vector<TreeT>;

  Visualization(ros::NodeHandle* nh)
      : tree_vis_(std::make_unique<TreeVisualizationT>()),
        path_vis_(std::make_unique<PathVisualization>(nh)) {}

  virtual ~Visualization() = default;

  inline void updatePathVisualization(const PlanT& plan) {
    path_vis_->setPath(plan);
  }

  /**
   * @brief
   * @param trees
   */
  virtual void updateSearchTreeVisualization(const TreeVector& trees) = 0;

  inline void publishVisualization() const {
    path_vis_->publishVisualization();
    tree_vis_->publishVisualization();
  }

  inline void clearVisualization() {
    path_vis_->clearVisualization();
    tree_vis_->clearVisualization();
  }

 protected:
  // Search tree visualization utility
  TreeVisualizationPtr tree_vis_;

 private:
  // Path visualization utility
  PathVisualizationPtr path_vis_;
};
}  // namespace rrt_planner::visualization

#endif
