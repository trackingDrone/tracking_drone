cmake_minimum_required(VERSION 2.8.3)
project(iarc_missions)
set(OpenCV_DIR "/home/gjx/opencv/opencv-3.4.4/build")   

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  dji_sdk_lib
  geometry_msgs
  roscpp
  rospy
  std_msgs
  iarc_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
 generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
  actionlib_msgs
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${iarc_missions_SOURCE_DIR}/include/iarc_missions
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${tracking_SOURCE_DIR} include/tracking_test
)
add_executable(
	srtp_mission_node 
	src/iarc_mission_node.cpp 
	src/iarc_mission.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(srtp_mission_node iarc_mission_generate_messages_cpp)
add_dependencies(srtp_mission_node dji_sdk_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(srtp_mission_node
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)