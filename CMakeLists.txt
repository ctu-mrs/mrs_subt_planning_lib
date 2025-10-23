cmake_minimum_required(VERSION 3.20.0)
project(mrs_subt_planning_lib)

# Set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)
if(${USE_ROS_TIMER})
  message(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  octomap_msgs
  octomap
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  Eigen3
  PCL
  mrs_lib
  mrs_modules_msgs
  mrs_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_modules_msgs_INCLUDE_DIRS}
  ${mrs_msgs_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------
# Disables annoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# Your library
add_library(${PROJECT_NAME} SHARED
  src/pcl_map.cpp
  src/astar_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(include)

# Link libraries using imported targets where applicable
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rclcpp::rclcpp
    Eigen3::Eigen
    mrs_lib
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------
# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Include headers for installation
install(DIRECTORY include/mrs_subt_planning_lib/
  DESTINATION include
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} TARGETS ${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
