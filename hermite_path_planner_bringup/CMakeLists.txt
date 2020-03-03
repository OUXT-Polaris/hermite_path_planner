cmake_minimum_required(VERSION 3.5)
project(hermite_path_planner_bringup)

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
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hermite_path_planner REQUIRED)
find_package(hermite_path_msgs REQUIRED)
find_package(pure_pursuit_planner REQUIRED)
find_package(velocity_planner REQUIRED)
find_package(quaternion_operation REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(hermite_path_planner_bringup_node src/hermite_path_planner_bringup.cpp)
ament_target_dependencies(hermite_path_planner_bringup_node
  rclcpp hermite_path_msgs hermite_path_planner pure_pursuit_planner velocity_planner quaternion_operation)

install(TARGETS
  hermite_path_planner_bringup_node
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Install ##
#############
install(DIRECTORY launch
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

ament_package()
