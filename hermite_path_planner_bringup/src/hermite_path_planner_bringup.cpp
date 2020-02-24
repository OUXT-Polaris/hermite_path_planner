#include <rclcpp/rclcpp.hpp>
#include <hermite_path_planner/hermite_path_planner_component.h>
#include <pure_pursuit_planner/pure_pursuit_planner_component.h>
#include <velocity_planner/velocity_planner_component.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto hermite_path_planner = std::make_shared<hermite_path_planner::HermitePathPlannerComponent>(options);
  auto pure_pursuit_planner = std::make_shared<pure_pursuit_planner::PurePursuitPlannerComponent>(options);
  auto velocity_planner = std::make_shared<velocity_planner::VelocityPlannerComponent>(options);
  exec.add_node(hermite_path_planner);
  exec.add_node(pure_pursuit_planner);
  exec.add_node(velocity_planner);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}