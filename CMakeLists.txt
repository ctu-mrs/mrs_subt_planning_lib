cmake_minimum_required(VERSION 2.8.3)
project(darpa_planning_lib)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  Boost REQUIRED COMPONENTS
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  mrs_lib
  std_msgs
  pcl_conversions
  pcl_ros
  octomap_ros
  )

find_package(PCL REQUIRED COMPONENTS)

###############################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(DIRECTORY msg FILES
#   )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(DEPENDENCIES
#   )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs 
  DEPENDS PCL
  LIBRARIES DarpaPlanningLib 
  )

###########
## Build ##
###########

include_directories(
  INCLUDE_DIRS include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(DarpaPlanningLib
  src/astar_planner.cpp
  src/pcl_map.cpp
  src/darpa_explorer.cpp
  )

add_dependencies(DarpaPlanningLib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(DarpaPlanningLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

#############
## Install ##
#############

# install(TARGETS DarpaPlanningLibrary
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   )
