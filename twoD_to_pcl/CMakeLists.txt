cmake_minimum_required(VERSION 3.0.2)
project(twoD_to_pcl)

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_ros
  laser_geometry
  #tf
  lidar2gps_msgs
)



## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES twoD_to_pcl
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs pcl_ros laser_geometry  lidar2gps_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/twoD_to_pcl.cpp) 


target_link_libraries(${PROJECT_NAME}_node 
${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

