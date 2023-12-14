cmake_minimum_required(VERSION 3.10)
project(rrt)

add_compile_options(-std=c++17)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

set(library_name rrt_plugin)

find_package(ompl REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED 
  COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  roscpp
  visualization_msgs
  tf2
  nav_utils
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${library_name}_2d
    ${library_name}_hybrid
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    visualization_msgs
    tf2
    nav_utils
)

include_directories(
  include
  # ${nav_utils_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${library_name}_2d SHARED
  src/rrt_planner/planner_plugin/rrt_plugin_2d.cpp
  src/rrt_planner/planner_core/rrt.cpp
  src/rrt_planner/planner_core/bidirectional_rrt.cpp
  src/rrt_planner/visualization_plugin/search_tree_visualization.cpp
  src/state_space/state_connector_2d/line_connector.cpp
  src/state_space/state_connector_2d/state_connector_2d.cpp
  src/state_space/state_connector_hybrid/analytic_motion_hybrid.cpp
  src/state_space/state_connector_hybrid/state_connector_hybrid.cpp
)

add_library(${library_name}_hybrid SHARED
  src/rrt_planner/planner_plugin/rrt_plugin_hybrid.cpp
  src/rrt_planner/planner_core/rrt.cpp
  src/rrt_planner/planner_core/bidirectional_rrt.cpp
  src/rrt_planner/visualization_plugin/search_tree_visualization.cpp
  src/state_space/state_connector_2d/line_connector.cpp
  src/state_space/state_connector_2d/state_connector_2d.cpp
  src/state_space/state_connector_hybrid/analytic_motion_hybrid.cpp
  src/state_space/state_connector_hybrid/state_connector_hybrid.cpp
)

# target_link_libraries(${library_name}_2d ${nav_utils_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${library_name}_2d ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_dependencies(${library_name}_2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${library_name}_hybrid ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_dependencies(${library_name}_hybrid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
