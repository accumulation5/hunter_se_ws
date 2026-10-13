cmake_minimum_required(VERSION 3.10.2)
project(amass_global_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_msgs
    pluginlib
    roscpp
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/global_planner_core.cpp
  src/astar_planner.cpp
  src/pso_planner.cpp
  src/rrt_planner.cpp
  src/improved_rrt_planner.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS amass_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)