cmake_minimum_required(VERSION 2.8.3)
project(fsm_visual_behavior)

set (CMAKE_CXX_STANDARD 17)

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
  OpenCV
  tf2_geometry_msgs
  tf2
  tf2_ros
  geometry_msgs
  message_generation
)

add_message_files(
  FILES 
  bbx_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometry_tf
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
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(fsm_visual_behavior 
  src/fsm_visual_behavior/PIDController.cpp
  /home/adrian/Escritorio/catking_ws/src/software_arq_robots_course/geometry_tf/src/geometry_tf/transforms.cpp
)

set(LINK_LIBS
  fsm_visual_behavior
  ${catkin_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_ball_behavior_tree 
  src/nodo_ball_behavior_tree.cpp
  src/fsm_visual_behavior/Ball_Detected.cpp
  src/fsm_visual_behavior/Turn.cpp
  src/fsm_visual_behavior/Follow_Ball.cpp
  src/fsm_visual_behavior/Person_Detected.cpp
  src/fsm_visual_behavior/Follow_Person.cpp
)
target_link_libraries(nodo_ball_behavior_tree ${catkin_LIBRARIES})
add_dependencies(nodo_ball_behavior_tree fsm_visual_behavior_generate_messages_cpp)

add_executable(nodo_camera_filtered src/nodo_camera_filtered.cpp)
target_link_libraries(nodo_camera_filtered ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nodo_bbx_msg_publisher src/nodo_bbx_msg_publisher)
target_link_libraries(nodo_bbx_msg_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(nodo_bbx_msg_publisher fsm_visual_behavior_generate_messages_cpp)

add_executable(nodo_tf_sub_rgbd 
	src/nodo_tf_sub_rgbd.cpp
)
target_link_libraries(nodo_tf_sub_rgbd ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")


roslint_cpp(
  src/nodo_filter.cpp
 
)
