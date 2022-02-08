cmake_minimum_required(VERSION 3.5)
project(sim_bridge LANGUAGES CXX)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(catkin REQUIRED 
  roscpp
  trajectory_msgs
  raisim
  quadruped_msgs
  quadruped_ctrl
  roslib
  tf
  geometry_msgs
  COMPONENTS)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS
   roscpp
   raisim
   roslib
   quadruped_msgs
   message_runtime
   quadruped_ctrl
   DEPENDS)

include_directories (
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

set (minicheetah_src
    src/ros_controller.cpp
    src/minicheetah.cpp
    include/ros_controller.h )
add_executable(minicheetah ${minicheetah_src})
target_link_libraries(minicheetah PUBLIC pthread ${catkin_LIBRARIES} Eigen3::Eigen)

add_dependencies(minicheetah quadruped_msgs_gencfg)
