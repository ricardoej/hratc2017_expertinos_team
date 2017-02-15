cmake_minimum_required(VERSION 2.8.3)
project(convert_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES convert_odom
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(convert_odom src/convert_odom.cpp src/convert_odom_node.cpp)
target_link_libraries(convert_odom ${catkin_LIBRARIES})
