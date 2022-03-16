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
  /home/elisa/catkin_ws/src/software_arq_robots_course/geometry_tf/src/geometry_tf
  )

set(LINK_LIBS
  fsm_visual_behavior
  ${catkin_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_camera_filtered src/nodo_camera_filtered.cpp)
target_link_libraries(nodo_camera_filtered ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nodo_tf_sub_rgbd 
	src/nodo_tf_sub_rgbd.cpp
)
target_link_libraries(nodo_tf_sub_rgbd ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")


roslint_cpp(
  src/nodo_filter.cpp
 
)
