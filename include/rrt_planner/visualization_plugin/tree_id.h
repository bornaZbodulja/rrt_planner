/**
 * @file tree_id.h
 * @author Borna Zbodulja (borna.zbodulja@gmail.com)
 * @brief Tree id utility
 * @version 0.1
 * @date 2024-04-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RRT_PLANNER__VISUALIZATION_PLUGIN__TREE_ID_H_
#define RRT_PLANNER__VISUALIZATION_PLUGIN__TREE_ID_H_

namespace rrt_planner::visualization {

enum class TreeId { ROOT_TREE = 0, TARGET_TREE = 1 };

inline const char* treeIdToString(TreeId tree_id) {
  switch (tree_id) {
    case TreeId::ROOT_TREE:
      return "root_tree";
    case TreeId::TARGET_TREE:
      return "target_tree";
  }

  __builtin_unreachable();
}
}  // namespace rrt_planner::visualization

#endif  // RRT_PLANNER__VISUALIZATION_PLUGIN__TREE_ID_H_
