cmake_minimum_required(VERSION 3.20.0)
project(mrs_subt_planning_lib)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  Eigen3
  cmake_modules
  mrs_lib
  octomap_msgs
  octomap_ros
  PCL
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf
)

set(LIBRARIES
  MrsSubtPlanningLib_Planner
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_msgs_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

add_library(MrsSubtPlanningLib_Planner SHARED
  src/astar_planner.cpp
  src/pcl_map.cpp
)

ament_target_dependencies(MrsOctomapServer
  ${DEPENDENCIES}
)

## components 
rclcpp_components_register_nodes(MrsSubtPlanningLib_Planner PLUGIN "MrsSubtPlanningLib_Planner::MrsSubtPlanningLib_Planner" EXECUTABLE MrsSubtPlanningLib_Planner)  # a modifier

target_compile_definitions(MrsSubtPlanningLib_Planner PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

ament_export_include_directories(include)

target_link_libraries(MrsSubtPlanningLib_Planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES} 
)

ament_package()