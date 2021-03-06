cmake_minimum_required(VERSION 2.8.3)
project(kinect2_tracker)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  kdl_conversions
  roscpp
  roslib
  tf
  tf_conversions
  std_msgs
  pcl_ros
  pcl_conversions
  openni2_camera
)

find_package(Eigen REQUIRED)
find_package(cmake_modules REQUIRED)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
)

add_message_files(
   FILES
   user_IDs.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect2_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions roscpp roslib tf tf_conversions
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(kinect2_tracker
#   src/${PROJECT_NAME}/kinect2_tracker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kinect2_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(NITE2_DIR /home/Dev/NiTE-2.2)
set(NITE2_LIB /home/Dev/NiTE-2.2/Redist/libNiTE2.so)
set(OPENNI2_DIR /home/Dev/OpenNI-2.2)
set(OPENNI2_LIB /home/Dev/OpenNI-2.2/Redist/libOpenNI2.so)

include_directories(${OPENNI2_DIR}/Include)
include_directories(${NITE2_DIR}/Include)
include_directories(./include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

add_executable(kinect2_tracker_node src/kinect2_tracker_node.cpp)
target_link_libraries(kinect2_tracker_node ${OPENNI2_LIB}  ${NITE2_LIB}  ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(frame_creator_node src/frame_creator_node.cpp)
target_link_libraries(frame_creator_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(vrep_publisher_node src/vrep_publisher_node.cpp)
target_link_libraries(vrep_publisher_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})