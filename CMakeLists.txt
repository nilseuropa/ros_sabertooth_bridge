cmake_minimum_required(VERSION 2.8.3)
project(sabertooth_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs serial geometry_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sabertooth_node src/sabertooth_node.cpp)

target_link_libraries(sabertooth_node
   ${catkin_LIBRARIES}
)
