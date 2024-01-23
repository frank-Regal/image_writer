cmake_minimum_required(VERSION 3.10.1)
project(image_writer)

find_package(catkin REQUIRED COMPONENTS
  roslib
  cv_bridge
  roscppsu
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge opencv roscpp rospy sensor_msgs
)

## Find other required packages
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare C++ libraries
add_library(ImageWriter include/Private/ImageWriter.cpp)
target_link_libraries(ImageWriter ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Declare ROS executable
add_executable(save_raw_image_stream src/save_raw_image_stream.cpp)
target_link_libraries(save_raw_image_stream ${catkin_LIBRARIES} ${OpenCV_LIBS} ImageWriter)

## Declare python nodes
catkin_install_python(PROGRAMS scripts/rosbag_manip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


