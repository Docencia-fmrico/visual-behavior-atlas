cmake_minimum_required(VERSION 2.8.3)
project(fsm_visual_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  geometry_tf
  behaviortree_cpp_v3
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS   
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  geometry_tf
  behaviortree_cpp_v3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(fsm_visual_behavior 
  src/fsm_visual_behavior/PIDController.cpp
  /home/alumnos/josemjim/catkin_ws/src/software_arq_robots_course/geometry_tf/src/geometry_tf
  )

set(LINK_LIBS
  fsm_visual_behavior
  ${catkin_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(behaviorTreePerson
  src/behaviorTreePerson.cpp
  src/fsm_visual_behavior/Person/FollowPerson.cpp
  src/fsm_visual_behavior/Person/PersonNotFound.cpp
  src/fsm_visual_behavior/Person/SearchPerson.cpp
)

target_link_libraries(behaviorTreePerson ${catkin_LIBRARIES})

add_executable(nodo_camera_filtered src/nodo_camera_filtered.cpp)
target_link_libraries(nodo_camera_filtered ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nodo_tf_sub_rgbd 
	src/nodo_tf_sub_rgbd.cpp
)
target_link_libraries(nodo_tf_sub_rgbd ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")


roslint_cpp(
  src/nodo_filter.cpp
  src/behaviorTreePerson.cpp
  include/fsm_visual_behavior/Person/FollowPerson.h src/fsm_visual_behavior/Person/FollowPerson.cpp
  include/fsm_visual_behavior/Person/PersonNotFound.h src/fsm_visual_behavior/PersonNotFound.cpp
  include/fsm_visual_behavior/Person/SearchPerson.h src/fsm_visual_behavior/Person/SearchPerson.cpp
)
