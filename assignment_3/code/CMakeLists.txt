cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(robofinal)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
)

find_package(OpenCV)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  .
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_LIBRARIES})

catkin_package(
  INCLUDE_DIRS
    .
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    tf
    geometry_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
)

add_executable(robofinal main.cpp MyRobot.cpp Utils.cpp)
target_link_libraries(robofinal ${catkin_LIBRARIES})
target_link_libraries(robofinal ${OpenCV_LIBRARIES})

install(TARGETS robofinal RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
