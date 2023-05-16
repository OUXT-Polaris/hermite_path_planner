#include <chrono>
#include <hermite_path_planner/hermite_path_generator.hpp>
#include <iostream>
#include <vector>
#include <velocity_planner/velocity_graph.hpp>
#include <velocity_planner/velocity_planning.hpp>

int main()
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(100);
  double acceleration_limit = 10;
  double deceleration_limit = -10;
  double velocity_limit = 2.0;
  for (int i = 0; i < 100; i++) {
    constraints[i].t = i;
    constraints[i].linear_velocity = 1;
    constraints[i].stop_flag = false;
  }
  constraints[99].stop_flag = true;
  {
    geometry_msgs::msg::Pose start_pose, goal_pose;
    goal_pose.position.x = 99.0;
    std::chrono::system_clock::time_point start, end;
    hermite_path_planner::HermitePathGenerator generator(1);
    const auto path = generator.generateHermitePath(start_pose, goal_pose);
    hermite_path_msgs::msg::HermitePathStamped path_stamped;
    path_stamped.path = path;
    path_stamped.reference_velocity = constraints;
    start = std::chrono::system_clock::now();
    velocity_planner::VelocityGraph graph(
      path_stamped, 0.1, acceleration_limit, deceleration_limit, velocity_limit);
    graph.getPlan();
    end = std::chrono::system_clock::now();
    std::cout << "A* method, elapsed:"
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us"
              << std::endl;
  }
  {
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();
    velocity_planning::planVelocity(
      constraints, acceleration_limit, deceleration_limit, velocity_limit);
    end = std::chrono::system_clock::now();
    std::cout << "Non A* method, elapsed:"
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us"
              << std::endl;
  }
  return 0;
}