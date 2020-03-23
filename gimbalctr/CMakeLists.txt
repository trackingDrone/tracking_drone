cmake_minimum_required(VERSION 2.8.3)
project(mk_gimbalctr)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  iarc_msgs
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${mk_gimbalctr_SOURCE_DIR} include/mk_gimbalctr
)

add_executable(
  mk_gimbalctr 
  src/mk_gimbalCtr_main.cpp 
  src/mk_gimbalCtr.cpp 
  src/mk_gimbalActions.cpp
)
target_link_libraries(mk_gimbalctr ${catkin_LIBRARIES})

add_dependencies(mk_gimbalctr ${catkin_EXPORTED_TARGETS} )

catkin_package(
)


