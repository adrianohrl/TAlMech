cmake_minimum_required(VERSION 2.8.3)
project(talmech)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  variant_topic_tools
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES talmech
  CATKIN_DEPENDS roscpp std_msgs variant_topic_tools
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CPP_FILES
)

set(H_FILES
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/talmech.cpp
)