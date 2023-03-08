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
#include <velocity_planner/velocity_planning.hpp>

namespace velocity_planning
{
hermite_path_msgs::msg::ReferenceVelocity VelocityConstraint::toRos() const
{
  hermite_path_msgs::msg::ReferenceVelocity new_constraint;
  new_constraint.t = t;
  new_constraint.linear_velocity = v_limit;
  return new_constraint;
}

bool isAllConstraintsIsChecked(const std::vector<VelocityConstraint> & constraints)
{
  for (const auto & constraint : constraints) {
    if (!constraint.isChecked()) {
      return false;
    }
  }
  return true;
}

size_t getMinimumVelocityLimitIndex(const std::vector<VelocityConstraint> & constraints)
{
  if (constraints.empty()) {
    throw std::runtime_error("Size of the constraints are zero.");
  }
  if (isAllConstraintsIsChecked(constraints)) {
    throw std::runtime_error("All constraints was checked!");
  }
  double min_vel = std::numeric_limits<double>::infinity();
  for (const auto & constraint : constraints) {
    if (!constraint.isChecked() && min_vel > constraint.v_limit) {
      min_vel = constraint.v_limit;
    }
  }
  size_t index = 0;
  for (const auto & constraint : constraints) {
    if (min_vel == constraint.v_limit) {
      return index;
    }
    index++;
  }
  throw std::runtime_error("Failed to find index");
}

std::vector<size_t> getAdjacentIndex(
  const std::vector<VelocityConstraint> & constraints, size_t index)
{
  if (index >= constraints.size()) {
    throw std::runtime_error("Index shoome_exceptionuld be under size of constraints.");
  }
  if (index == 0) {
    return {index + 1};
  } else if (index == (constraints.size() - 1)) {
    return {index - 1};
  } else {
    return {index - 1, index + 1};
  }
}

double getAcceleration(const VelocityConstraint & v1, const VelocityConstraint & v2)
{
  if (v1.t > v2.t) {
    return (v1.v_limit * v1.v_limit - v2.v_limit * v2.v_limit) / (2 * (v1.t - v2.t));
  } else {
    return (v2.v_limit * v2.v_limit - v1.v_limit * v1.v_limit) / (2 * (v2.t - v1.t));
  }
}

double getSatisfiedVelocity(
  const double acceleration_limit, const double deceleration_limit, const VelocityConstraint & from,
  const VelocityConstraint & to)
{
  double acceleration = (to.v * to.v - from.v * from.v) / (2 * (to.t - from.t));
  if (acceleration > 0) {
    return std::sqrt(
      from.v * from.v +
      2 * std::min(std::abs(acceleration), std::abs(acceleration_limit)) * (to.t - from.t));

  } else {
    return std::sqrt(
      to.v * to.v -
      2 * std::min(std::abs(acceleration), std::abs(deceleration_limit)) * (from.t - to.t));
  }
}

void clampVelocityConstraint(std::vector<VelocityConstraint> & constraints, double velocity_limit)
{
  for (auto & constraint : constraints) {
    if (constraint.v > velocity_limit) {
      constraint.v = velocity_limit;
    }
  }
}

void updateAdjacentVelocity(
  std::vector<VelocityConstraint> & constraints, const double acceleration_limit,
  const double deceleration_limit, size_t target_index)
{
  constraints[target_index].check();
  for (const auto index : getAdjacentIndex(constraints, target_index)) {
    if (!constraints[index].isChecked()) {
      constraints[index].v = getSatisfiedVelocity(
        acceleration_limit, deceleration_limit, constraints[std::min(index, target_index)],
        constraints[std::max(index, target_index)]);
      if (std::isnan(constraints[index].v)) {
        throw std::runtime_error("The velocity is none.");
      }
    }
  }
}

size_t getMinimumIndex(const std::vector<VelocityConstraint> & constraints)
{
  double min = std::numeric_limits<double>::infinity();
  double min_index = constraints.size() - 1;
  for (size_t i = 0; i < constraints.size(); i++) {
    if (!constraints[i].isChecked() && min > constraints[i].v) {
      min = constraints[i].v;
      min_index = i;
    }
  }
  return min_index;
}

std::vector<VelocityConstraint> planVelocity(
  std::vector<VelocityConstraint> constraints, double acceleration_limit, double deceleration_limit,
  double velocity_limit)
{
  clampVelocityConstraint(constraints, velocity_limit);
  updateAdjacentVelocity(constraints, acceleration_limit, deceleration_limit, constraints.size() - 1);
  while (!isAllConstraintsIsChecked(constraints)) {
    updateAdjacentVelocity(constraints, acceleration_limit, deceleration_limit, getMinimumIndex(constraints));
  }
  return constraints;
}

std::vector<VelocityConstraint> fromRos(
  const std::vector<hermite_path_msgs::msg::ReferenceVelocity> & constraints)
{
  std::vector<VelocityConstraint> converted_constraints;
  std::transform(
    constraints.begin(), constraints.end(), std::back_inserter(converted_constraints),
    [](const hermite_path_msgs::msg::ReferenceVelocity & constraint) {
      return VelocityConstraint(constraint);
    });
  return converted_constraints;
}

std::vector<hermite_path_msgs::msg::ReferenceVelocity> toRos(
  const std::vector<VelocityConstraint> & constraints)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> converted_velocity;
  std::transform(
    constraints.begin(), constraints.end(), std::back_inserter(converted_velocity),
    [](const VelocityConstraint & constraint) { return constraint.toRos(); });
  return converted_velocity;
}

std::vector<hermite_path_msgs::msg::ReferenceVelocity> planVelocity(
  const std::vector<hermite_path_msgs::msg::ReferenceVelocity> & constraints,
  double acceleration_limit, double deceleration_limit, double velocity_limit)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> filtered_constraints;
  for (const auto & constraint : constraints) {
    filtered_constraints.emplace_back(constraint);
    if (constraint.stop_flag) {
      break;
    };
  }
  return toRos(planVelocity(
    fromRos(filtered_constraints), acceleration_limit, deceleration_limit, velocity_limit));
}
}  // namespace velocity_planning