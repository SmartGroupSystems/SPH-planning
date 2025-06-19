cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Acceleration.msg
  BsplineTraj.msg
  Force.msg
  Odom.msg
  OdomBroadcast.msg
  OdomWithNeighbors.msg
  Particle.msg
  Position.msg
  PositionCommand.msg
  Swarm_particles.msg
  Swarm_traj.msg
  SwarmData.msg
  Velocity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  DEPENDS message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

