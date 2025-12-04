cmake_minimum_required(VERSION 3.20)
project(mrs_subt_planning_lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[example_controller_plugin]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  ament_cmake
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  octomap_msgs
  octomap
  mrs_msgs
  PCL
  Eigen3
  pluginlib
  mrs_lib
  )


set(LIBRARIES
  mrs_subt_planning_lib
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(mrs_subt_planning_lib SHARED
  src/pcl_map.cpp
  src/astar_planner.cpp
)

# ROS2 / ament deps (adds include dirs & links)
ament_target_dependencies(mrs_subt_planning_lib
  ${DEPENDENCIES}
)

## | -------------------- export interfaces ------------------- |

foreach(library IN LISTS LIBRARIES)
  target_include_directories(${library}
    PUBLIC
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()

add_library(mrs_subt_planning_lib_mrs_subt_planning_lib INTERFACE)

# add_library(mrs_subt_planning_lib::mrs_subt_planning_lib ALIAS mrs_subt_planning_lib_mrs_subt_planning_lib)

set_target_properties(mrs_subt_planning_lib_mrs_subt_planning_lib
  PROPERTIES
    EXPORT_NAME mrs_subt_planning_lib
    OUTPUT_NAME mrs_subt_planning_lib
)

target_link_libraries(mrs_subt_planning_lib_mrs_subt_planning_lib
  INTERFACE
    ${LIBRARIES}
)

target_include_directories(mrs_subt_planning_lib_mrs_subt_planning_lib
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# add_executable(mrs_subt_planning_lib
#   src/astar_planner.cpp
#   src/pcl_map.cpp
# )

# target_link_libraries(mrs_subt_planning_lib PRIVATE mrs_subt_planning_lib)

# target_link_libraries(mrs_subt_planning_lib
#   PRIVATE
#     rclcpp::rclcpp
#     ${std_msgs_TARGETS}
#     mrs_lib::mrs_lib
#     PCL::PCL  
# )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS
    mrs_subt_planning_lib_mrs_subt_planning_lib
    ${LIBRARIES}
    ${EXECUTABLES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION "include/${PROJECT_NAME}"
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()