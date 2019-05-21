cmake_minimum_required(VERSION 2.8.3)
project(adas_module)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED)
find_package(OpenCV)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG -fopenmp")

## Generate messages in the 'msg' folder
 add_message_files(
    FILES 
    track.msg
    TrackArray.msg
    ld_LaneParam.msg
    ld_Frame.msg
    ld_Point.msg
    ld_Coeff.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
)

# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
   common.h
   func_lcw.h
)

include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare a C++ executable -- ros_fusion for fusing all sensor data
 add_executable(adas_module 
    src/main.cpp
    src/common.cpp
    src/func_lcw.cpp
    )

 add_dependencies(adas_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
 target_link_libraries(adas_module
   ${catkin_LIBRARIES}
 )
target_link_libraries(adas_module ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++ )

