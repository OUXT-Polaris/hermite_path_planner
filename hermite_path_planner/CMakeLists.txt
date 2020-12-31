cmake_minimum_required(VERSION 3.5)
project(hermite_path_planner)

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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(hermite_path_generator SHARED
  src/hermite_path_generator.cpp
)
ament_target_dependencies(hermite_path_generator
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  quaternion_operation
  hermite_path_msgs
  color_names)
ament_export_targets(export_hermite_path_generator HAS_LIBRARY_TARGET)
ament_export_libraries(hermite_path_generator)

add_library(hermite_path_planner_component SHARED
  src/hermite_path_planner_component.cpp
  src/hermite_path_generator.cpp)
target_compile_definitions(hermite_path_planner_component
  PRIVATE "HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(hermite_path_planner_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  quaternion_operation
  visualization_msgs
  color_names
  tf2_ros
  tf2_geometry_msgs
  hermite_path_msgs)
rclcpp_components_register_nodes(hermite_path_planner_component
  "hermite_path_planner::HermitePathPlannerComponent")

add_executable(hermite_path_planner_node
  src/hermite_path_planner_node.cpp)
ament_target_dependencies(hermite_path_planner_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  quaternion_operation
  visualization_msgs
  color_names
  tf2_ros
  tf2_geometry_msgs
  hermite_path_msgs)
target_link_libraries(hermite_path_planner_node hermite_path_planner_component)

install(TARGETS
  hermite_path_planner_node
  DESTINATION lib/hermite_path_planner
)
install(TARGETS hermite_path_planner_component hermite_path_generator
EXPORT export_hermite_path_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_libraries(hermite_path_planner_component)

# Install header files
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# launch/conifg file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
