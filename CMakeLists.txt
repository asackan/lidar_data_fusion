cmake_minimum_required(VERSION 3.10.2)
project(lidar_fusion CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(laser_fusion src/main.cpp src/lidar_fusion.cpp)
target_link_libraries(laser_fusion ${catkin_LIBRARIES})