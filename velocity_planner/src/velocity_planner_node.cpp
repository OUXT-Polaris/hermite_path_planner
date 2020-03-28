#include <velocity_planner/velocity_planner_component.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<velocity_planner::VelocityPlannerComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}