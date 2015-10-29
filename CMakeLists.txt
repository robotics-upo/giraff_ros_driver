cmake_minimum_required(VERSION 2.8.3)

project(giraff_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  avr_comms.msg
  batteries.msg
  cmd_vel_avr.msg
  Pilot.msg
  Stalk.msg
  StalkRef.msg
)

generate_messages(
   DEPENDENCIES
   nav_msgs   sensor_msgs   std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES giraff_ros_driver
 CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf message_runtime
 DEPENDS system_lib
)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(giraff_node src/giraff_node.cpp)
add_executable(giraff_teleop_joy src/giraff_teleop_joy.cpp)

target_link_libraries(giraff_node
   ${catkin_LIBRARIES}
)

target_link_libraries(giraff_teleop_joy
   ${catkin_LIBRARIES}
)
