cmake_minimum_required(VERSION 3.12.0)
project(scoutair_planner)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type )

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/exploration_manager.cpp
  src/frontier_map.cpp
  src/perception_utils.cpp
  src/raycast.cpp
  src/voxblox_map.cpp
  src/frontier_visulization.cpp
  src/graph_node.cpp
  src/astar2.cpp
  src/exploration_fsm.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(exploration_manager_node
  src/exploration_manager_node.cpp
)
target_link_libraries(exploration_manager_node ${PROJECT_NAME})

cs_add_executable(exploration_fsm_node
  src/exploration_fsm_node.cpp
)
target_link_libraries(exploration_fsm_node ${PROJECT_NAME})


cs_install()
cs_export()


