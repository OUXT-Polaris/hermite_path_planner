// Copyright (c) 2023 OUXT Polaris
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

/**
 * @file test_velocity_planning.cpp
 * @author Kento Hirogaki hkt8g2r6kin@gmail.com
 * @brief test code for Velocity Planning
 * @version 0.1
 * @date 2023-03-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <gtest/gtest.h>
#include <velocity_planner/velocity_planning.hpp>

TEST(TestSuite, testCase1)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(4);
  double acceleration_limit = 10;
  double deceleration_limit = -10;
  double velocity_limit = 2.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  constraints[0].t = 1;
  constraints[0].linear_velocity = 1;
  constraints[0].stop_flag = false;
  constraints[1].t = 2;
  constraints[1].linear_velocity = 2;
  constraints[1].stop_flag = false;
  constraints[2].t = 3;
  constraints[2].linear_velocity = 1;
  constraints[2].stop_flag = false;
  constraints[3].t = 4;
  constraints[3].linear_velocity = 0;
  constraints[3].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);

  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_DOUBLE_EQ(converted_constraints[0].linear_velocity, 1);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_DOUBLE_EQ(converted_constraints[1].linear_velocity, 2);
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 1);
  EXPECT_EQ(converted_constraints[2].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[3].t, 4);
  EXPECT_DOUBLE_EQ(converted_constraints[3].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[3].stop_flag, true);
}

TEST(TestSuite, testCase2)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(3);
  double acceleration_limit = 20;
  double deceleration_limit = -20;
  double velocity_limit = 3.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  constraints[0].t = 1;
  constraints[0].linear_velocity = 0;
  constraints[0].stop_flag = false;
  constraints[1].t = 2;
  constraints[1].linear_velocity = 3;
  constraints[1].stop_flag = false;
  constraints[2].t = 3;
  constraints[2].linear_velocity = 0;
  constraints[2].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);

  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_DOUBLE_EQ(converted_constraints[0].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_DOUBLE_EQ(converted_constraints[1].linear_velocity, 3);
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[2].stop_flag, true);
}

TEST(TestSuite, testCase3)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(3);
  double acceleration_limit = 20;
  double deceleration_limit = -20;
  double velocity_limit = 2.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  constraints[0].t = 1;
  constraints[0].linear_velocity = 0;
  constraints[0].stop_flag = false;
  constraints[1].t = 2;
  constraints[1].linear_velocity = 3;
  constraints[1].stop_flag = false;
  constraints[2].t = 3;
  constraints[2].linear_velocity = 0;
  constraints[2].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);
  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_DOUBLE_EQ(converted_constraints[0].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_DOUBLE_EQ(
    converted_constraints[1].linear_velocity, 2);
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[2].stop_flag, true);
}

TEST(TestSuite, testCase4)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(3);
  double acceleration_limit = 3.0;
  double deceleration_limit = -20.0;
  double velocity_limit = 20.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  constraints[0].t = 1;
  constraints[0].linear_velocity = 0;
  constraints[0].stop_flag = false;
  constraints[1].t = 2;
  constraints[1].linear_velocity = 3;
  constraints[1].stop_flag = false;
  constraints[2].t = 3;
  constraints[2].linear_velocity = 0;
  constraints[2].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);
  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_DOUBLE_EQ(converted_constraints[0].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_EQ(
    std::abs(converted_constraints[1].linear_velocity - std::sqrt(6)) < 0.0001,
    true);
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[2].stop_flag, true);
}

TEST(TestSuite, testCase5)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(3);
  double acceleration_limit = 3.0;
  double deceleration_limit = -3.0;
  double velocity_limit = 20.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  constraints[0].t = 1;
  constraints[0].linear_velocity = 4.8;
  constraints[0].stop_flag = false;
  constraints[1].t = 2;
  constraints[1].linear_velocity = 0;
  constraints[1].stop_flag = false;
  constraints[2].t = 3;
  constraints[2].linear_velocity = 0;
  constraints[2].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);
  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_EQ(
    std::abs(converted_constraints[0].linear_velocity - std::sqrt(6)) < 0.0001,
    true);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_DOUBLE_EQ(converted_constraints[1].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[2].stop_flag, true);
}

TEST(TestSuite, testCase6)
{
  int constraints_num = 100;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(constraints_num);
  double acceleration_limit = 3.0;
  double deceleration_limit = -3.0;
  double velocity_limit = 20.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  for (int i = 0; i < constraints_num; i++) {
    constraints[i].t = i;
    constraints[i].linear_velocity = 0;
    constraints[i].stop_flag = false;
  }
  constraints[constraints_num - 1].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);
  for (int i = 0; i < constraints_num - 1; i++) {
    EXPECT_DOUBLE_EQ(converted_constraints[i].t, i);
    EXPECT_DOUBLE_EQ(converted_constraints[i].linear_velocity, 0);
    EXPECT_EQ(converted_constraints[i].stop_flag, false);
  }
  EXPECT_DOUBLE_EQ(converted_constraints[constraints_num - 1].t, constraints_num - 1);
  EXPECT_DOUBLE_EQ(converted_constraints[constraints_num - 1].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[constraints_num - 1].stop_flag, true);
}

TEST(TestSuite, testCase7)
{
  int constraints_num = 26;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(constraints_num);
  std::vector<double> linear_velocity = {0.0, 3.0, 4.0, 6.0, 2.0, 8.0, 6.0, 6.0, 4.0,
                                         0.0, 0.0, 1.0, 1.0, 0.0, 7.0, 8.0, 4.0, 8.0,
                                         9.0, 4.0, 3.0, 9.0, 4.0, 5.0, 7.0, 0.0};
  double acceleration_limit = 10.0;
  double deceleration_limit = -10.0;
  double velocity_limit = 8.0;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;

  for (int i = 0; i < constraints_num; i++) {
    constraints[i].t = i;
    constraints[i].linear_velocity = linear_velocity[i];
    constraints[i].stop_flag = false;
  }
  constraints[constraints_num].stop_flag = true;
  converted_constraints = velocity_planning::planVelocity(
    constraints, acceleration_limit, deceleration_limit, velocity_limit);

  for (int i = 0; i < constraints_num; i++) {
    EXPECT_EQ(converted_constraints[i].linear_velocity <= velocity_limit, true);
  }
  for (int i = 0; i < constraints_num - 1; i++) {
    EXPECT_EQ(
      std::abs(
        converted_constraints[i + 1].linear_velocity *
          converted_constraints[i + 1].linear_velocity -
        converted_constraints[i].linear_velocity * converted_constraints[i].linear_velocity) /
          (2 * (converted_constraints[i + 1].t - converted_constraints[i].t)) <=
        acceleration_limit + 0.000001,
      true);
  }
}

/**
 * @brief Run all the tests that were declared with TEST()
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}