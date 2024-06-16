cmake_minimum_required(VERSION 3.12.0)
project(scoutair_planner)

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type -Wno-dev )

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   geometry_msgs
#   message_generation
#   roscpp
#   std_msgs
#   tf
#   visualization_msgs
#   voxblox_ros
# )

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/frontier_map.cpp
  src/perception_utils.cpp
  src/raycast.cpp
  src/voxblox_map.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(frontier_map_node
  src/frontier_map_node.cpp
)
target_link_libraries(frontier_map_node ${PROJECT_NAME})


