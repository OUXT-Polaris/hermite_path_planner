#include <hermite_path_planner/hermite_path_planner_component.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<hermite_path_planner::HermitePathPlannerComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
