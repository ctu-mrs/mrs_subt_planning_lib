cmake_minimum_required(VERSION 3.20)
project(mrs_subt_planning_lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  )


set(LIBRARIES
  MrsSubtPlanningLib_Planner
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

add_library(MrsSubtPlanningLib_Planner
  src/pcl_map.cpp
  src/astar_planner.cpp
)

target_include_directories(MrsSubtPlanningLib_Planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ROS2 / ament deps (adds include dirs & links)
ament_target_dependencies(MrsSubtPlanningLib_Planner
  ${DEPENDENCIES}
)

set_target_properties(MrsSubtPlanningLib_Planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS MrsSubtPlanningLib_Planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
