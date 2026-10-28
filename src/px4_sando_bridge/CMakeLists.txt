cmake_minimum_required(VERSION 3.16)
project(px4_sando_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynus_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# Frame transforms library
add_library(px4_sando_transforms
  src/frame_transforms.cpp
)
target_include_directories(px4_sando_transforms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(px4_sando_transforms PUBLIC
  tf2::tf2
)

# Watchdog and clock-synchronisation helpers
add_library(px4_sando_core
  src/timestamp_sync.cpp
  src/watchdog.cpp
)
target_include_directories(px4_sando_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Bridge node executable
add_executable(px4_sando_bridge_node
  src/main.cpp
  src/px4_sando_bridge_node.cpp
)
target_include_directories(px4_sando_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(px4_sando_bridge_node
  px4_sando_transforms
  px4_sando_core
  rclcpp::rclcpp
  ${dynus_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${px4_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
  target_link_libraries(test_frame_transforms px4_sando_transforms)

  ament_add_gtest(test_watchdog test/test_watchdog.cpp)
  target_link_libraries(test_watchdog px4_sando_core)

  ament_add_gtest(test_timestamp_sync test/test_timestamp_sync.cpp)
  target_link_libraries(test_timestamp_sync px4_sando_core)
endif()

# Installation
install(TARGETS
  px4_sando_transforms
  px4_sando_core
  px4_sando_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(px4_sando_transforms px4_sando_core)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  dynus_interfaces
  px4_msgs
)

ament_package()
