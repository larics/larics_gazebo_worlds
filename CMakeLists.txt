cmake_minimum_required(VERSION 2.8.3)
project(larics_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS
    roslib
    roscpp
    octomap_msgs
    tf
    )

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES larics_gazebo_worlds
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(octomap REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(transform_octomap src/transform_octomap.cpp)
target_link_libraries(transform_octomap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} yaml-cpp)
link_libraries(${OCTOMAP_LIBRARIES})