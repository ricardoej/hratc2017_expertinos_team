cmake_minimum_required(VERSION 2.8.3)
project(hratc2017_entry_template)

find_package(catkin REQUIRED COMPONENTS
  metal_detector_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES hratc2016_entry_template
  CATKIN_DEPENDS metal_detector_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(hratc2016_entry_template_node src/hratc2016_entry_template_node.cpp)
add_executable(random_walk_navigation_node examples/cpp/random_walk_navigation.cpp)
add_executable(random_walk_simple_node examples/cpp/random_walk_simple.cpp)
add_executable(read_metal_detector_node examples/cpp/read_metal_detector.cpp)
add_executable(set_mine_node examples/cpp/set_mine.cpp)
add_executable(sweep_node_simulation examples/cpp/sweep_sim.cpp)
add_executable(sweep_node_robot examples/cpp/sweep_robot.cpp)
add_executable(get_global_odom_node examples/cpp/get_global_odom.cpp)

add_dependencies(read_metal_detector_node metal_detector_msgs_generate_messages_py)

target_link_libraries(random_walk_navigation_node ${catkin_LIBRARIES})
target_link_libraries(random_walk_simple_node ${catkin_LIBRARIES})
target_link_libraries(read_metal_detector_node ${catkin_LIBRARIES})
target_link_libraries(set_mine_node ${catkin_LIBRARIES})
target_link_libraries(sweep_node_simulation ${catkin_LIBRARIES})
target_link_libraries(sweep_node_robot ${catkin_LIBRARIES})
target_link_libraries(get_global_odom_node ${catkin_LIBRARIES})
