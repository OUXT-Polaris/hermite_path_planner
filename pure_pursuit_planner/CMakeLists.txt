cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_planner)

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
find_package(hermite_path_planner REQUIRED)
find_package(hermite_path_msgs REQUIRED)

include_directories(
  include
)

add_library(pure_pursuit_planner_component SHARED 
  src/pure_pursuit_planner_component.cpp)
target_compile_definitions(pure_pursuit_planner_component
  PRIVATE "PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(pure_pursuit_planner_component
  rclcpp 
  rclcpp_components 
  hermite_path_msgs)
rclcpp_components_register_nodes(pure_pursuit_planner_component
  "pure_pursuit_planner::PurePursuitPlannerComponent")

add_executable(pure_pursuit_planner_node
  src/pure_pursuit_planner_node.cpp
)
target_link_libraries(pure_pursuit_planner_node pure_pursuit_planner_component)
ament_target_dependencies(pure_pursuit_planner_node
  rclcpp 
  rclcpp_components 
  hermite_path_msgs)

install(TARGETS
  pure_pursuit_planner_node
  DESTINATION lib/pure_pursuit_planner
)
install(TARGETS pure_pursuit_planner_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

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

ament_package()
