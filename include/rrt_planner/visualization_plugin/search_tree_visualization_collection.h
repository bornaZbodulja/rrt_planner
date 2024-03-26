/**
 * @file search_tree_visualization_collection.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Utility for visualizing multiple search trees
 * @version 0.1
 * @date 2023-10-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_COLLECTION_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_COLLECTION_H_

#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <std_msgs/ColorRGBA.h>

#include <map>
#include <memory>
#include <vector>

#include "rrt_planner/visualization_plugin/search_tree_visualization.h"

namespace rrt_planner::visualization {
class SearchTreeVisualizationCollection {
 public:
  SearchTreeVisualizationCollection() = default;
  ~SearchTreeVisualizationCollection() = default;

  /**
   * @brief
   * @param nh
   * @param tree_id
   * @param tree_color
   */
  void addTreeVisualization(ros::NodeHandle* nh, const std::string& tree_id,
                            const std_msgs::ColorRGBA& tree_color) {
    collection_.try_emplace(tree_id, std::make_unique<SearchTreeVisualization>(
                                         nh, tree_id, tree_color));
  }

  void setTreeVisualization(
      const std::string& tree_id,
      const std::vector<std::vector<geometry_msgs::Pose>>& search_tree) {
    auto it = collection_.find(tree_id);
    if (it == collection_.end()) {
      // Visualization for given id doesn't exist
      return;
    }

    it->second->setTree(search_tree);
  }

  void publishVisualization() const {
    std::for_each(collection_.cbegin(), collection_.cend(),
                  [&](const auto& it) { it.second->publishVisualization(); });
  }

  void clearVisualization() {
    std::for_each(collection_.cbegin(), collection_.cend(),
                  [&](const auto& it) { it.second->clearVisualization(); });
  }

 private:
  std::map<std::string, std::unique_ptr<SearchTreeVisualization>> collection_{};
};
}  // namespace rrt_planner::visualization

#endif  // RRT_PLANNER__VISUALIZATION_PLUGIN__SEARCH_TREE_VISUALIZATION_COLLECTION_H_
