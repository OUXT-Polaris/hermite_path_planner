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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

class VelocityConstraint
{
public:
  VelocityConstraint(double t, double v_limit)
  : t(t), v_limit(v_limit), v(v_limit), is_checked_(false)
  {
  }
  const double t;        // position in frenet coordinage
  const double v_limit;  // velocity constraint
  double v;
  void check() { is_checked_ = true; }
  bool isChecked() const { return is_checked_; }

private:
  bool is_checked_;
};

bool isAllConstraintsIsChecked(const std::vector<VelocityConstraint> & constraints);
size_t getMinimumVelocityLimitIndex(const std::vector<VelocityConstraint> & constraints);
std::vector<size_t> getAdjacentIndex(
  const std::vector<VelocityConstraint> & constraints, size_t index);
double getAcceleration(const VelocityConstraint & v1, const VelocityConstraint & v2);
double getSatisfiedVelocity(
  const double acceleration_limit, const VelocityConstraint & from, const VelocityConstraint & to);
void updateAdjacentVelocity(
  std::vector<VelocityConstraint> & constraints, const double acceleration_limit,
  size_t target_index);
size_t getMinimumIndex(const std::vector<VelocityConstraint> & constraints);
std::vector<VelocityConstraint> getVelocityConstraint(
  std::vector<VelocityConstraint> constraints, double acceleration_limit, double velocity_limit);