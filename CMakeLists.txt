cmake_minimum_required(VERSION 2.8.3)
project(fsm_visual_person)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  behaviortree_cpp_v3
)

catkin_package(
CATKIN_DEPENDS   
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(LINK_LIBS
  fsm_visual_person
  ${catkin_LIBRARIES}
)

add_executable(nodo_behavior_person
  src/person_node.cpp
  src/fsm_visual_person/FollowPerson.cpp
  src/fsm_visual_person/PersonFound.cpp
  src/fsm_visual_person/SearchPerson.cpp
)

target_link_libraries(nodo_behavior_person ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

roslint_cpp(
  src/person_node.cpp
  include/fsm_visual_person/FollowPerson.h src/fsm_visual_person/FollowPerson.cpp
  include/fsm_visual_person/PersonFound.h src/fsm_visual_person/PersonFound.cpp
  include/fsm_visual_person/SearchPerson.h src/fsm_visual_person/SearchPerson.cpp
)