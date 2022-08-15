cmake_minimum_required(VERSION 3.5)
project(mrs_subt_planning_lib)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

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
  LIBRARIES MrsSubtPlanningLib 
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

add_library(MrsSubtPlanningLib
  src/astar_planner.cpp
  src/pcl_map.cpp
  )

add_dependencies(MrsSubtPlanningLib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsSubtPlanningLib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

#############
## Install ##
#############

# install(TARGETS MrsSubtPlanningLibrary
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   )
