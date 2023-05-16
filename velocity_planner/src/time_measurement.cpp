#include <iostream>
#include <vector>
#include <chrono>
#include <velocity_planner/velocity_planning.hpp>
int main()
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(100);
  double acceleration_limit = 10;
  double deceleration_limit = -10;
  double velocity_limit = 2.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> astar_converted_constraints;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> new_converted_constraints;
  for (int i = 0; i < 100; i++) {
    constraints[i].t = i;
    constraints[i].linear_velocity = 1;
    constraints[i].stop_flag = false;
  }
  constraints[99].stop_flag = true;
  astar_converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);

  std::chrono::system_clock::time_point start, end;  // 型は auto で可
  start = std::chrono::system_clock::now();          // 計測開始時間
  velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);
  
  end = std::chrono::system_clock::now();  // 計測終了時間
  double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                     .count();  //処理に要した時間をミリ秒に変換
  std::cout << "elapsed:" << elapsed << "us" << std::endl;
  return 0;
}