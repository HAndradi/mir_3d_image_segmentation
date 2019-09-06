cmake_minimum_required(VERSION 2.8.3)
project(mir_3d_image_segmentation)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
  std_msgs
)

# Compile as C++11 for ROS Kinetic and newer
add_compile_options(-std=c++11
  -O3
  -march=native
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  ros/msg
  FILES
  Pixel_Coord.msg
  Cluster_Pixels.msg
  Clusters.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(segment ros/src/segment.cpp)
target_link_libraries(segment ${catkin_LIBRARIES})
add_executable(closest_obj ros/src/closest_obj.cpp)
target_link_libraries(closest_obj ${catkin_LIBRARIES})
