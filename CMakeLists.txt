cmake_minimum_required(VERSION 3.10.1)
project(image_writer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DHL2SS_ENABLE_ZDEPTH)

find_package(catkin REQUIRED COMPONENTS
  roslib
  cv_bridge
  roscpp
  rospy
  std_msgs
  sensor_msgs
  audio_common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge opencv roscpp rospy sensor_msgs
)

## Find other required packages
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Include directories
include_directories(
  include
  third_party/hl2ss/extensions/client_cpp
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  third_party/hl2ss/3rdparty/Zdepth/include
  third_party/hl2ss/3rdparty/Zdepth/zstd/include
  third_party/hl2ss/3rdparty/Zdepth/zstd/src
)

# Collect Zdepth source files
file(GLOB ZDEPTH_SOURCES
    "third_party/hl2ss/3rdparty/Zdepth/src/*.cpp"
    "third_party/hl2ss/3rdparty/Zdepth/zstd/src/*.c"
)

# Find the libsndfile library
find_library(SNDFILE_LIBRARY NAMES sndfile libsndfile HINTS "/usr/lib" "/usr/local/lib")
if(NOT SNDFILE_LIBRARY)
  message(FATAL_ERROR "libsndfile not found")
endif()

## Declare C++ libraries
add_library(ImageWriter include/Private/ImageWriter.cpp)
target_link_libraries(ImageWriter ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Declare C++ libraries
add_library(ImageWriterParams include/Private/ImageWriterParams.cpp)
target_link_libraries(ImageWriterParams ImageWriter ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(hl2ss_lib
  # third_party/hl2ss/extensions/client_cpp/main.cpp
  third_party/hl2ss/extensions/client_cpp/hl2ss.cpp
  third_party/hl2ss/extensions/client_cpp/hl2ss_lnm.cpp
  third_party/hl2ss/extensions/client_cpp/hl2ss_mt.cpp
  ${ZDEPTH_SOURCES}
)
target_link_libraries(hl2ss_lib
  ${OpenCV_LIBS}
  avcodec
  avutil
  pthread
)

# Add before your add_executable commands
target_link_directories(hl2ss_lib PRIVATE /usr/local/lib)

## Declare ROS executable
add_executable(save_raw_image_stream src/save_raw_image_stream.cpp)
target_link_libraries(save_raw_image_stream ${catkin_LIBRARIES} ${OpenCV_LIBS} ImageWriter ImageWriterParams)

add_executable(save_image_stream_hl2ss src/save_image_stream_hl2ss.cpp)
target_link_libraries(save_image_stream_hl2ss ${catkin_LIBRARIES} ${OpenCV_LIBS} ImageWriter ImageWriterParams hl2ss_lib)

add_executable(save_image_stream_hl2ss_mt src/save_image_stream_hl2ss_mt.cpp)
target_link_libraries(save_image_stream_hl2ss_mt ${catkin_LIBRARIES} ${OpenCV_LIBS} ImageWriter ImageWriterParams hl2ss_lib)

add_executable(save_image_stream_hl2ss_mt_int src/save_image_stream_hl2ss_mt_int.cpp)
target_link_libraries(save_image_stream_hl2ss_mt_int ${catkin_LIBRARIES} ${OpenCV_LIBS} ImageWriter ImageWriterParams hl2ss_lib)

add_executable(save_raw_audio_stream_hl2ss src/save_raw_audio_stream_hl2ss.cpp)
target_link_libraries(save_raw_audio_stream_hl2ss ${catkin_LIBRARIES} ${SNDFILE_LIBRARY} ${OpenCV_LIBS} hl2ss_lib)

add_executable(test_audio src/test_audio.cpp)
target_link_libraries(test_audio ${catkin_LIBRARIES} ${SNDFILE_LIBRARY} ${OpenCV_LIBS} hl2ss_lib)

# target_compile_options(ImageWriter PRIVATE -O3)
# target_compile_options(ImageWriterParams PRIVATE -O3)
# target_compile_options(hl2ss_lib PRIVATE -O3)

# target_compile_options(save_raw_image_stream PRIVATE
#     -O3
#     -fdiagnostics-color=always
#     -g
# )

# target_compile_options(save_image_stream_hl2ss PRIVATE
#     -O3
#     -fdiagnostics-color=always
#     -g
# )

# target_compile_options(save_image_stream_hl2ss_mt PRIVATE
#     -O3
#     -fdiagnostics-color=always
#     -g
# )


## Declare python nodes
catkin_install_python(PROGRAMS scripts/rosbag_manip.py scripts/video_writer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


