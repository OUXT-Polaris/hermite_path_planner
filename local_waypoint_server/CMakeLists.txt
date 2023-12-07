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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

generate_parameter_library(obstacle_waypoint
  config/obstacle_waypoint.yaml
)

ament_auto_add_library(local_waypoint_server_component SHARED
  src/local_waypoint_server_component.cpp)
target_compile_definitions(local_waypoint_server_component
  PRIVATE "LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_BUILDING_DLL")
rclcpp_components_register_nodes(local_waypoint_server_component
  "local_waypoint_server::LocalWaypointServerComponent")
target_link_libraries(local_waypoint_server_component obstacle_waypoint)

ament_auto_add_executable(local_waypoint_server_node
  src/local_waypoint_server_node.cpp
)
target_link_libraries(local_waypoint_server_node obstacle_waypoint local_waypoint_server_component)

# launch/conifg file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
