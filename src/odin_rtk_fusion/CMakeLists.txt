cmake_minimum_required(VERSION 3.10)
project(odin_rtk_fusion VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build-time configuration
set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_ID
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT GIT_COMMIT_ID)
  set(GIT_COMMIT_ID "unknown")
endif()
message(STATUS "Git commit ID: ${GIT_COMMIT_ID}")

configure_file(
  ${PROJECT_SOURCE_DIR}/modules/common/build_config.h.in
  ${CMAKE_BINARY_DIR}/generated/common/build_config.h
)

# ROS version: 1 for catkin (ROS1), 2 for ament (ROS2)
# Auto-detect from the $ROS_VERSION env var exported by /opt/ros/<distro>/setup.bash.
# Can be overridden with -DROS_VERSION=1|2.
if(DEFINED ENV{ROS_VERSION} AND NOT "$ENV{ROS_VERSION}" STREQUAL "")
  set(_ROS_VERSION_DEFAULT "$ENV{ROS_VERSION}")
else()
  set(_ROS_VERSION_DEFAULT "2")
endif()
set(ROS_VERSION "${_ROS_VERSION_DEFAULT}" CACHE STRING "ROS version (1=catkin, 2=ament)")
set_property(CACHE ROS_VERSION PROPERTY STRINGS "1" "2")
message(STATUS "ROS_VERSION=${ROS_VERSION} (env ROS_VERSION=$ENV{ROS_VERSION})")

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

if(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nmea_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(pcl_conversions REQUIRED)
else()
  find_package(catkin REQUIRED COMPONENTS
    roscpp nav_msgs geometry_msgs sensor_msgs
    nmea_msgs tf2_ros visualization_msgs pcl_ros pcl_conversions
  )
  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs
      nmea_msgs tf2_ros visualization_msgs pcl_ros pcl_conversions
    DEPENDS GTSAM
  )

  # include_directories($<$<NOT:$<EQUAL:${ROS_VERSION},2>>:${catkin_INCLUDE_DIRS}>)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(
  modules
  ${CMAKE_BINARY_DIR}/generated
  ${GTSAM_INCLUDE_DIR}
  ${PROJ_INCLUDE_DIRS}
)

add_library(fusion_algo STATIC
  modules/fusion/gtsam/graph_fusion.cpp
  modules/fusion/alignment/alignment_fusion.cpp
  modules/fusion/fusion_factory.cpp
  modules/fusion/odom_buffer.cpp
  modules/fusion/yaw_init.cpp
  modules/common/logger.cpp
)
target_link_libraries(fusion_algo
  spdlog::spdlog 
  gtsam 
  yaml-cpp
  ${PROJ_LIBRARIES}
  )

add_executable(fusion_node ros/fusion_node.cpp)
target_link_libraries(fusion_node fusion_algo)

if(ROS_VERSION EQUAL 2)
  ament_target_dependencies(fusion_node
    rclcpp nav_msgs geometry_msgs sensor_msgs
    nmea_msgs tf2_ros visualization_msgs pcl_ros pcl_conversions
  )
  target_compile_definitions(fusion_node PRIVATE ODIN_ROS2=1)

  install(TARGETS fusion_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY ros/launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY ros/rviz DESTINATION share/${PROJECT_NAME}/ros)
  ament_package()
else()
  target_link_libraries(fusion_node ${catkin_LIBRARIES})

  # bin_replay_node: replay ODINBIN files in ROS1
  add_executable(bin_replay_node ros/bin_replay_node.cpp)
  target_link_libraries(bin_replay_node fusion_algo ${catkin_LIBRARIES})

  install(TARGETS fusion_node bin_replay_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  install(DIRECTORY ros/rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros)
endif()