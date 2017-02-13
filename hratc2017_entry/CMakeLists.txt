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
  include/hratc2017/coils.h
  include/hratc2017/map.h
  include/utilities/enum_converter.h
  include/utilities/exception.h
  include/utilities/ros_node.h
)

set(CPP_FILES
  src/hratc2017/coils.cpp
  src/hratc2017/map.cpp
  src/utilities/exception.cpp
  src/utilities/ros_node.cpp
)

add_executable(landmine_analyzer_node src/landmine_analyzer_node.cpp src/hratc2017/landmine_analyzer.cpp ${CPP_FILES})
add_dependencies(landmine_analyzer_node metal_detector_msgs_generate_messages_py)
target_link_libraries(landmine_analyzer_node ${catkin_LIBRARIES})

add_executable(metal_scanner_node src/metal_scanner_node.cpp src/hratc2017/metal_scanner.cpp ${CPP_FILES})
add_dependencies(metal_scanner_node metal_detector_msgs_generate_messages_py)
target_link_libraries(metal_scanner_node ${catkin_LIBRARIES})

add_executable(waypoints_controller_node src/waypoints_controller_node.cpp src/hratc2017/waypoints_controller.cpp ${CPP_FILES})
add_dependencies(waypoints_controller_node metal_detector_msgs_generate_messages_py)
target_link_libraries(waypoints_controller_node ${catkin_LIBRARIES})
