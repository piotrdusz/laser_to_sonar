cmake_minimum_required(VERSION 3.0.2)
project(laser_to_sonar)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_to_sonar
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_to_sonar src/main.cpp)
target_link_libraries(laser_to_sonar ${catkin_LIBRARIES})
