// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <hermite_path_planner/hermite_path_planner_component.hpp>
#include <pure_pursuit_planner/pure_pursuit_planner_component.hpp>
#include <velocity_planner/curve_planner_component.hpp>
#include <velocity_planner/obstacle_planner_component.hpp>
#include <velocity_planner/planner_concatenator_component.hpp>
#include <velocity_planner/stop_planner_component.hpp>
#include <velocity_planner/velocity_planner_component.hpp>
#include <local_waypoint_server/local_waypoint_server_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto hermite_path_planner =
    std::make_shared<hermite_path_planner::HermitePathPlannerComponent>(options);
  auto pure_pursuit_planner =
    std::make_shared<pure_pursuit_planner::PurePursuitPlannerComponent>(options);
  auto curve_planner = std::make_shared<velocity_planner::CurvePlannerComponent>(options);
  auto obstacle_planner = std::make_shared<velocity_planner::ObstaclePlannerComponent>(options);
  auto velocity_planner = std::make_shared<velocity_planner::VelocityPlannerComponent>(options);
  auto stop_planner = std::make_shared<velocity_planner::StopPlannerComponent>(options);
  auto planner_concatenator =
    std::make_shared<velocity_planner::PlannerConcatenatorComponent>(options);
  auto local_waypoint_server =
    std::make_shared<local_waypoint_server::LocalWaypointServerComponent>(options);
  exec.add_node(hermite_path_planner);
  exec.add_node(pure_pursuit_planner);
  exec.add_node(curve_planner);
  exec.add_node(obstacle_planner);
  exec.add_node(velocity_planner);
  exec.add_node(stop_planner);
  exec.add_node(planner_concatenator);
  exec.add_node(local_waypoint_server);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
