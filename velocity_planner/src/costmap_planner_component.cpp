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

#include <limits>
#include <memory>
#include <string>
#include <velocity_planner/costmap_planner_component.hpp>

namespace velocity_planner
{
CostmapPlannerComponent::CostmapPlannerComponent(const rclcpp::NodeOptions & options)
: Node("costmap_planner", "velocity_planner", options), generator_(0.0), viz_(get_name())
{
  std::string hermite_path_topic;
  std::string costmap_topic;
  declare_parameter("hermite_path_topic", "/hermite_path_planner/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  declare_parameter("costmap_topic", "/perception/") hermite_path_sub_ =
    this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
      hermite_path_topic, 1,
      std::bind(&CostmapPlannerComponent::hermitePathCallback, this, std::placeholders::_1));
}

void CostmapPlannerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{
  path_ = *data;
}

}  // namespace velocity_planner
