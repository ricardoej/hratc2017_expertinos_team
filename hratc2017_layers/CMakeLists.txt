cmake_minimum_required(VERSION 2.8.3)
project(hratc2017_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  hratc2017_entry
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hratc2017_layers ${hratc2017_entry_LIBRARIES}
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
)

include_directories(
  include
  ${hratc2017_entry_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(hratc2017_layers
  src/hratc2017/covered_area_layer.cpp
  src/hratc2017/landmines_layer.cpp
  src/hratc2017/obstacles_layer.cpp
)

target_link_libraries(hratc2017_layers ${catkin_LIBRARIES} ${hratc2017_entry_LIBRARIES})
