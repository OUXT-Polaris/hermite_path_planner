#include <pure_pursuit_planner/pure_pursuit_planner_component.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<pure_pursuit_planner::PurePursuitPlannerComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}