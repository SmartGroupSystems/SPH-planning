cmake_minimum_required(VERSION 3.0.2)
project(water_swarm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  common_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy common_msgs
)

include_directories(
  include/water_swarm
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_executable(sph_vis src/sph_vis.cpp)
target_link_libraries(sph_vis ${catkin_LIBRARIES})
add_dependencies(sph_vis ${catkin_EXPORTED_TARGETS})  

add_executable(sph_zhang_3d src/sph_zhang_3d.cpp)
target_link_libraries(sph_zhang_3d ${catkin_LIBRARIES})
add_dependencies(sph_zhang_3d ${catkin_EXPORTED_TARGETS}) 
