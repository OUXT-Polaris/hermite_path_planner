// Copyright (c) 2019 OUXT Polaris
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
 * @brief test code for Quaternion Operation
 * @version 0.1
 * @date 2019-04-21
 *
 * @copyright Copyright (c) 2019
 *
 */

// headers in Google Test
#include <gtest/gtest.h>

// headers in this package
#include <velocity_planner/velocityplanning.hpp>

/**
 * @brief test for + operator
 *
 */
TEST(TestSuite, testCase1)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> constraints;
  double acceleration_limit = 10;
  double deceleration_limit = -10;
  double velocity_limit = 10;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_constraints;
  
  constraints[0].t = 1;
  constraints[0].linear_velocity = 1;
  constraints[0].stop_flag = false;

  constraints[1].t = 1;
  constraints[1].linear_velocity = 1;
  constraints[1].stop_flag = false;

  constraints[2].t = 1;
  constraints[2].linear_velocity = 1;
  constraints[2].stop_flag = false;

  constraints[3].t = 1;
  constraints[3].linear_velocity = 1;
  constraints[3].stop_flag = false;

  converted_constraints = planVelocity(constraints, acceleration_limit, deceleration_limit, velocity_limit);
  EXPECT_EQ(converted_constraints[0].t , 0);
  EXPECT_EQ(converted_constraints[0].linear_velocity , 0);
  EXPECT_EQ(converted_constraints[0].stop_flag , 0);
  
  EXPECT_EQ(converted_constraints[1].t , 0);
  EXPECT_EQ(converted_constraints[1].linear_velocity , 0);
  EXPECT_EQ(converted_constraints[1].stop_flag , 0);
  
  EXPECT_EQ(converted_constraints[2].t , 0);
  EXPECT_EQ(converted_constraints[2].linear_velocity , 0);
  EXPECT_EQ(converted_constraints[2].stop_flag , 0);

  EXPECT_EQ(converted_constraints[3].t , 0);
  EXPECT_EQ(converted_constraints[3].linear_velocity , 0);
  EXPECT_EQ(converted_constraints[3].stop_flag , 0);
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