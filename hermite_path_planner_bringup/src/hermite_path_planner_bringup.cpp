#include <rclcpp/rclcpp.hpp>
#include <hermite_path_planner/hermite_path_planner_component.h>
#include <pure_pursuit_planner/pure_pursuit_planner_component.h>
#include <velocity_planner/velocity_planner_component.h>
#include <velocity_planner/curve_planner_component.h>
#include <velocity_planner/obstacle_planner_component.h>
#include <velocity_planner/stop_planner_component.h>
#include <velocity_planner/planner_concatenator_component.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto hermite_path_planner = std::make_shared<hermite_path_planner::HermitePathPlannerComponent>(options);
  auto pure_pursuit_planner = std::make_shared<pure_pursuit_planner::PurePursuitPlannerComponent>(options);
  auto curve_planner = std::make_shared<velocity_planner::CurvePlannerComponent>(options);
  auto obstacle_planner = std::make_shared<velocity_planner::ObstaclePlannerComponent>(options);
  auto velocity_planner = std::make_shared<velocity_planner::VelocityPlannerComponent>(options);
  auto stop_planner = std::make_shared<velocity_planner::StopPlannerComponent>(options);
  auto planner_concatenator = std::make_shared<velocity_planner::PlannerConcatenatorComponent>(options);
  exec.add_node(hermite_path_planner);
  exec.add_node(pure_pursuit_planner);
  exec.add_node(curve_planner);
  exec.add_node(obstacle_planner);
  exec.add_node(velocity_planner);
  exec.add_node(stop_planner);
  exec.add_node(planner_concatenator);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}