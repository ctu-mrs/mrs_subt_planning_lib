cmake_minimum_required(VERSION 3.20)
project(mrs_subt_planning_lib)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree octree)

add_library(${PROJECT_NAME}
  src/pcl_map.cpp
  src/astar_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# PCL defines/incs/libs
if (PCL_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})
  target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()

# Eigen
if (TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
elseif (DEFINED EIGEN3_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})
endif()

# ROS2 / ament deps (adds include dirs & links)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  octomap_msgs
  octomap
  visualization_msgs
  mrs_msgs
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
