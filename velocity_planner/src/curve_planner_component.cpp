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

#include <color_names/color_names.hpp>
#include <velocity_planner/curve_planner_component.hpp>
#include <memory>
#include <limits>
#include <string>

namespace velocity_planner
{
CurvePlannerComponent::CurvePlannerComponent(const rclcpp::NodeOptions & options)
: Node("curve_planner", "velocity_planner", options), generator_(0.0), viz_(get_name())
{
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  hermite_path_pub_ =
    this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>("~/hermite_path", 1);
  std::string hermite_path_topic;
  declare_parameter("hermite_path_topic", "/hermite_path_planner/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  declare_parameter("num_resolution", 200);
  get_parameter("num_resolution", num_resolution_);
  declare_parameter("max_linear_velocity", 0.5);
  get_parameter("max_linear_velocity", max_linear_velocity_);
  declare_parameter("target_angular_velocity", 0.3);
  get_parameter("target_angular_velocity", target_angular_velocity_);
  hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    hermite_path_topic, 1,
    std::bind(&CurvePlannerComponent::hermitePathCallback, this, std::placeholders::_1));
}

void CurvePlannerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{
  constexpr double e = std::numeric_limits<double>::epsilon();
  path_ = *data;
  for (int i = 0; i < num_resolution_; i++) {
    double t = (0.5 + static_cast<double>(i)) / num_resolution_;
    double curvature = generator_.getCurvature(path_->path, t);
    if (1 / std::fabs(curvature) * target_angular_velocity_ > max_linear_velocity_) {
      continue;
    }
    hermite_path_msgs::msg::ReferenceVelocity vel;
    vel.t = t;
    if (std::fabs(curvature) < e) {
      vel.linear_velocity = target_angular_velocity_;
    } else {
      vel.linear_velocity = boost::algorithm::clamp(
        1 / std::fabs(curvature) * target_angular_velocity_, 0.0, max_linear_velocity_);
    }
    vel.from_node = get_name();
    path_->reference_velocity.push_back(vel);
  }
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_.get(), color_names::makeColorMsg("lime", 1.0)));
  hermite_path_pub_->publish(path_.get());
}
}  // namespace velocity_planner
