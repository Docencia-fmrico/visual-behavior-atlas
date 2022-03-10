cmake_minimum_required(VERSION 2.8.3)
project(fsm_visual_behavior)

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
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(nodo_filter src/nodo_filter.cpp)
target_link_libraries(nodo_filter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")


roslint_cpp(
  src/nodo_filter.cpp
 
)
