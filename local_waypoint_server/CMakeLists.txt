cmake_minimum_required(VERSION 3.5)
project(local_waypoint_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hermite_path_planner REQUIRED)
find_package(hermite_path_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(local_waypoint_server_component SHARED
  src/local_waypoint_server_component.cpp)
target_compile_definitions(local_waypoint_server_component
  PRIVATE "LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(local_waypoint_server_component
  rclcpp
  rclcpp_components
  geometry_msgs
  hermite_path_planner
  hermite_path_msgs
  sensor_msgs
  quaternion_operation)
rclcpp_components_register_nodes(local_waypoint_server_component
  "local_waypoint_server::LocalWaypointServerComponent")

add_executable(local_waypoint_server_node
  src/local_waypoint_server_node.cpp
)
target_link_libraries(local_waypoint_server_node local_waypoint_server_component)
ament_target_dependencies(local_waypoint_server_node
  rclcpp
  rclcpp_components
  geometry_msgs
  hermite_path_planner
  hermite_path_msgs
  sensor_msgs
  quaternion_operation)

install(TARGETS
  local_waypoint_server_node
  DESTINATION lib/local_waypoint_server
)
install(TARGETS local_waypoint_server_component
EXPORT export_obstacle_avoid_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(local_waypoint_server_component)

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
)

# launch/conifg file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
