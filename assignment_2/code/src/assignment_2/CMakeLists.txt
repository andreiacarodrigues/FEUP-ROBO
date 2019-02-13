cmake_minimum_required(VERSION 2.8.3)
project(assignment_2)

find_package(catkin REQUIRED COMPONENTS 
roslib
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


set(CMAKE_BUILD_TYPE Release)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    .
  LIBRARIES
  CATKIN_DEPENDS
roslib
  roscpp
  tf
  stdr_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


add_executable(robot main.cpp line_following.h line_following.cpp)
add_dependencies(robot ${stdr_msgs_EXPORTED_TARGETS}) # wait for stdr_msgs to be build
target_link_libraries(robot
  ${catkin_LIBRARIES}
)
install(TARGETS robot
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

