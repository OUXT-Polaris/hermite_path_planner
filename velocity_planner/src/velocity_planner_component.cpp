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


#include <color_names/color_names.h>
#include <velocity_planner/velocity_graph.hpp>
#include <velocity_planner/velocity_planner_component.hpp>
#include <memory>
#include <string>

namespace velocity_planner
{
VelocityPlannerComponent::VelocityPlannerComponent(const rclcpp::NodeOptions & options)
: Node("velocity_planner", options), buffer_(get_clock()), listener_(buffer_), viz_(get_name())
{
  std::string current_twist_topic;
  declare_parameter("current_twist_topic", "current_twist");
  get_parameter("current_twist_topic", current_twist_topic);
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    current_twist_topic, 1,
    std::bind(&VelocityPlannerComponent::currentTwistCallback, this, std::placeholders::_1));

  std::string hermite_path_topic;
  declare_parameter("hermite_path_topic", "/planner_concatenator/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    hermite_path_topic, 1,
    std::bind(&VelocityPlannerComponent::hermitePathCallback, this, std::placeholders::_1));

  std::string current_pose_topic;
  declare_parameter("current_pose_topic", "current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&VelocityPlannerComponent::currentPoseCallback, this, std::placeholders::_1));

  hermite_path_pub_ =
    this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>("~/hermite_path", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);

  double robot_width;
  get_parameter("robot_width", robot_width);
  generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width);
}

bool VelocityPlannerComponent::checkTopics()
{
  if (path_ == boost::none) {
    RCLCPP_INFO(get_logger(), "path was not calculated");
    return false;
  }
  if (current_twist_ == boost::none) {
    RCLCPP_INFO(get_logger(), "current_twist did not published");
    return false;
  }
  if (current_pose_ == boost::none) {
    RCLCPP_INFO(get_logger(), "current_pose did not published");
    return false;
  }
  return true;
}

void VelocityPlannerComponent::checkCurrentPath()
{
  mtx_.lock();
  if (!checkTopics()) {
    mtx_.unlock();
    return;
  }
  mtx_.unlock();
}

void VelocityPlannerComponent::updatePath()
{
  mtx_.lock();
  if (!checkTopics()) {
    mtx_.unlock();
    return;
  }
  VelocityGraph graph(path_.get(), 0.05, 0.1, -0.1, 0.5);
  auto plan = graph.getPlan();
  if (plan) {
    RCLCPP_INFO(get_logger(), "velocity planning succeed");
    hermite_path_msgs::msg::HermitePathStamped path;
    path.path = path_->path;
    path.header = path_->header;
    path.reference_velocity = plan.get();
    hermite_path_pub_->publish(path);
    RCLCPP_INFO(
      get_logger(),
      "maximum acceleration is " + std::to_string(graph.getPlannedMaximumAcceleration()));
    RCLCPP_INFO(
      get_logger(),
      "minimum acceleration is " + std::to_string(graph.getPlannedMinimumAcceleration()));
    marker_pub_->publish(viz_.generateDeleteMarker());
    marker_pub_->publish(viz_.generatePolygonMarker(path, 0.0, 3.0));
    // marker_pub_->publish(viz_.generateMarker(path,color_names::makeColorMsg("royalblue",1.0)));
  } else {
    marker_pub_->publish(viz_.generateDeleteMarker());
    RCLCPP_INFO(get_logger(), "velocity planning failed");
    RCLCPP_WARN(get_logger(), graph.getReason());
  }
  mtx_.unlock();
}

void VelocityPlannerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{

  path_ = *data;
  updatePath();
}

void VelocityPlannerComponent::currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
{
  current_twist_ = *data;
}

void VelocityPlannerComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  current_pose_ = *data;
}
}  // namespace velocity_planner
