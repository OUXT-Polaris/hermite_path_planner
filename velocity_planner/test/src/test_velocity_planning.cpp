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

// headers in Google Test
#include <gtest/gtest.h>

// headers in this package
#include <velocity_planner/velocity_planning.hpp>

TEST(TestSuite, testCase1)
{
  double x;
  x = 1.0;
  EXPECT_DOUBLE_EQ(x, 1.0);
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
  double x;
  x = 2.0;
  EXPECT_DOUBLE_EQ(x, 2.0);
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
  double x;
  x = 3.0;
  EXPECT_DOUBLE_EQ(x, 3.0);
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints(3);
  double acceleration_limit = 20;
  double deceleration_limit = -20;
  double velocity_limit = 2.0;  // Changed from 3.0 in previous test
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
  // Verify that the converted_constraints have been modified by velocity_limit
  EXPECT_DOUBLE_EQ(converted_constraints[0].t, 1);
  EXPECT_DOUBLE_EQ(converted_constraints[0].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[0].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[1].t, 2);
  EXPECT_DOUBLE_EQ(
    converted_constraints[1].linear_velocity, 2);  // Expected value modified by velocity_limit
  EXPECT_EQ(converted_constraints[1].stop_flag, false);
  EXPECT_DOUBLE_EQ(converted_constraints[2].t, 3);
  EXPECT_DOUBLE_EQ(converted_constraints[2].linear_velocity, 0);
  EXPECT_EQ(converted_constraints[2].stop_flag, true);
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