cmake_minimum_required(VERSION 2.8.3)
project(hratc2017_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hratc2017_layers
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hratc2017_layers
  src/hratc2017/landmines_layer.cpp
  src/hratc2017/covered_area_layer.cpp
)

target_link_libraries(hratc2017_layers ${catkin_LIBRARIES})
