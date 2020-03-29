cmake_minimum_required(VERSION 3.5)
project(velocity_planner)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hermite_path_msgs REQUIRED)
find_package(hermite_path_planner REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(color_names REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERESS_INCLUDE_DIRS}
)

# Velocity Planner
add_library(velocity_planner_component SHARED
  src/velocity_planner_component.cpp
  src/velocity_visualizer.cpp
  src/velocity_graph.cpp)
target_compile_definitions(velocity_planner_component
  PRIVATE "VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_BUILDING_DLL")
target_link_libraries(velocity_planner_component ${CERES_LIBRARIES})
ament_target_dependencies(velocity_planner_component
  rclcpp
  rclcpp_components
  quaternion_operation
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  color_names
  hermite_path_planner
  hermite_path_msgs)
rclcpp_components_register_nodes(velocity_planner_component
  "velocity_planner::VelocityPlannerComponent")

add_executable(velocity_planner_node
  src/velocity_planner_node.cpp
)
target_link_libraries(velocity_planner_node velocity_planner_component)
ament_target_dependencies(velocity_planner_node
  color_names
  message_filters
  rclcpp
  rclcpp_components
  quaternion_operation
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  hermite_path_planner
  hermite_path_msgs)

# Curve Planner
add_library(curve_planner_component SHARED
  src/curve_planner_component.cpp
  src/velocity_visualizer.cpp
)
target_compile_definitions(curve_planner_component
  PRIVATE "VELOCITY_PLANNER_CURVE_PLANNER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(curve_planner_component
  rclcpp
  rclcpp_components
  quaternion_operation
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  color_names
  hermite_path_planner
  hermite_path_msgs)
rclcpp_components_register_nodes(curve_planner_component
  "velocity_planner::CurvePlannerComponent")

add_executable(curve_planner_node
  src/curve_planner_node.cpp
)
target_link_libraries(curve_planner_node curve_planner_component)
ament_target_dependencies(curve_planner_node
  rclcpp
  rclcpp_components
  quaternion_operation
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  color_names
  hermite_path_planner
  hermite_path_msgs)

# Obstacle Planner
add_library(obstacle_planner_component SHARED
  src/obstacle_planner_component.cpp
  src/velocity_visualizer.cpp
)
target_compile_definitions(obstacle_planner_component
  PRIVATE "VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(obstacle_planner_component
  rclcpp
  rclcpp_components
  sensor_msgs
  quaternion_operation
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  visualization_msgs
  color_names
  tf2_sensor_msgs
  hermite_path_planner
  hermite_path_msgs)
rclcpp_components_register_nodes(obstacle_planner_component
  "velocity_planner::ObstaclePlannerComponent")

add_executable(obstacle_planner_node
  src/obstacle_planner_node.cpp
)
target_link_libraries(obstacle_planner_node obstacle_planner_component)
ament_target_dependencies(obstacle_planner_node
  rclcpp
  rclcpp_components
  quaternion_operation
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  visualization_msgs
  color_names
  hermite_path_planner
  hermite_path_msgs)

# Velocity Constraint Planner
add_library(stop_planner_component SHARED
  src/stop_planner_component.cpp
  src/velocity_visualizer.cpp
)
target_compile_definitions(stop_planner_component
  PRIVATE "VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(stop_planner_component
  rclcpp
  rclcpp_components
  message_filters
  color_names
  quaternion_operation
  hermite_path_planner
  hermite_path_msgs)
rclcpp_components_register_nodes(stop_planner_component
  "velocity_planner::StopPlannerComponent")

add_executable(stop_planner_node
  src/stop_planner_node.cpp
)
target_link_libraries(stop_planner_node stop_planner_component)
ament_target_dependencies(stop_planner_node
  rclcpp
  rclcpp_components
  message_filters
  color_names
  quaternion_operation
  hermite_path_planner
  hermite_path_msgs)

# Planner Concatenator
add_library(planner_concatenator_component SHARED
  src/planner_concatenator_component.cpp
  src/velocity_visualizer.cpp
)
target_compile_definitions(planner_concatenator_component
  PRIVATE "VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_BUILDING_DLL")
ament_target_dependencies(planner_concatenator_component
  rclcpp
  rclcpp_components
  message_filters
  color_names
  quaternion_operation
  hermite_path_planner
  hermite_path_msgs)
rclcpp_components_register_nodes(planner_concatenator_component
  "velocity_planner::PlannerConcatenatorComponent")

add_executable(planner_concatenator_node
  src/planner_concatenator_node.cpp
)
target_link_libraries(planner_concatenator_node planner_concatenator_component)
ament_target_dependencies(planner_concatenator_node
  rclcpp
  rclcpp_components
  message_filters
  color_names
  quaternion_operation
  hermite_path_planner
  hermite_path_msgs)

# install
install(TARGETS
  obstacle_planner_node
  velocity_planner_node
  curve_planner_node
  stop_planner_node
  planner_concatenator_node
  DESTINATION lib/velocity_planner
)
install(TARGETS
  velocity_planner_component
  curve_planner_component
  obstacle_planner_component
  stop_planner_component
  planner_concatenator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(
  velocity_planner_component
  curve_planner_component
  obstacle_planner_component
  stop_planner_component
  planner_concatenator_component)

# launch/conifg file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
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
