#include <velocity_planner/planner_concatenator_component.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<velocity_planner::PlannerConcatenatorComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}