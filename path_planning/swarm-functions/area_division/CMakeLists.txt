cmake_minimum_required(VERSION 3.0.2)
project(area_division)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES area_division_lib
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(area_division_lib
  src/lib/area_division.cpp
  src/lib/connected_components.cpp
)

add_executable(area_division_node src/main.cpp)
target_link_libraries(area_division_node
  area_division_lib
  ${catkin_LIBRARIES}
)
