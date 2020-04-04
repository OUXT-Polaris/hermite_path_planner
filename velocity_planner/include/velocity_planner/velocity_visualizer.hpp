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

#ifndef VELOCITY_PLANNER__VELOCITY_VISUALIZER_HPP_
#define VELOCITY_PLANNER__VELOCITY_VISUALIZER_HPP_

#include <hermite_path_planner/hermite_path_generator.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <string>

namespace velocity_planner
{
class VelocityVisualizer
{
public:
  explicit VelocityVisualizer(std::string node_name);
  visualization_msgs::msg::MarkerArray generateMarker(
    hermite_path_msgs::msg::HermitePathStamped path);
  visualization_msgs::msg::MarkerArray generateMarker(
    hermite_path_msgs::msg::HermitePathStamped path, std_msgs::msg::ColorRGBA color_ref_velocity);
  visualization_msgs::msg::MarkerArray generateDeleteMarker();
  visualization_msgs::msg::MarkerArray generatePolygonMarker(
    hermite_path_msgs::msg::HermitePathStamped path, double ratio = 0.0, double width = 1.0);
  visualization_msgs::msg::MarkerArray generateObstacleMarker(
    double t, hermite_path_msgs::msg::HermitePathStamped path, std_msgs::msg::ColorRGBA color,
    double width = 5.0);

private:
  std::string node_name_;
  hermite_path_planner::HermitePathGenerator generator_;
  double getVelocity(
    std::vector<hermite_path_msgs::msg::ReferenceVelocity> vel, double t, double l);
};
}  // namespace velocity_planner

#endif  //  VELOCITY_PLANNER__VELOCITY_VISUALIZER_HPP_
