cmake_minimum_required(VERSION 2.8.3)
project(hratc2017_entry)

find_package(catkin REQUIRED COMPONENTS
  metal_detector_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  roscpp
  rospy
  tf
  message_filters
  move_base_msgs
  actionlib
  angles
  laser_assembler
  control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS metal_detector_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

set(H_FILES
  include/utilities/enum_converter.h
  include/utilities/exception.h
  include/utilities/ros_node.h
)

set(CPP_FILES
  src/utilities/exception.cpp
  src/utilities/ros_node.cpp
)

add_executable(metal_detector_node src/metal_detector_node.cpp src/entries/metal_detector_node.cpp ${CPP_FILES})
add_dependencies(metal_detector_node metal_detector_msgs_generate_messages_py)
target_link_libraries(metal_detector_node ${catkin_LIBRARIES})
