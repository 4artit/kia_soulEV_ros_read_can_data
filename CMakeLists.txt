cmake_minimum_required(VERSION 2.8.3)
project(ros_read_candata)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  can_msgs
)


catkin_package(
  LIBRARIES ros_read_candata
  CATKIN_DEPENDS message_generation roscpp std_msgs geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_read_candata.cpp
# )

add_executable(${PROJECT_NAME}_node src/ros_read_candata_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
