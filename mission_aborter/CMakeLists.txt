cmake_minimum_required(VERSION 3.0)
project(mission_aborter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp mavros_msgs swarmros)

catkin_package(
  INCLUDE_DIRS ./src /usr/include
  CATKIN_DEPENDS mavros_msgs swarmros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/mission_aborter.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

