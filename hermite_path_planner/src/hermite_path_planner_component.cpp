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

#include <hermite_path_planner/hermite_path_planner_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>

namespace hermite_path_planner
{
HermitePathPlannerComponent::HermitePathPlannerComponent(const rclcpp::NodeOptions & options)
: Node("hermite_path_planner", options), buffer_(get_clock()), listener_(buffer_)
{
  declare_parameter("planning_frame_id", "map");
  get_parameter("planning_frame_id", planning_frame_id_);
  declare_parameter("goal_pose_topic", "/move_base_simple/goal");
  get_parameter("goal_pose_topic", goal_pose_topic_);
  declare_parameter("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic_);
  declare_parameter("robot_width", 3.0);
  double robot_width;
  get_parameter("robot_width", robot_width);
  generator_ = std::make_shared<HermitePathGenerator>(robot_width);
  hermite_path_pub_ =
    this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>("~/hermite_path", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, 1,
    std::bind(&HermitePathPlannerComponent::GoalPoseCallback, this, std::placeholders::_1));
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic_, 1,
    std::bind(&HermitePathPlannerComponent::CurrentPoseCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::PoseStamped HermitePathPlannerComponent::TransformToPlanningFrame(
  geometry_msgs::msg::PoseStamped pose)
{
  if (pose.header.frame_id == planning_frame_id_) {
    return pose;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(pose.header.stamp.sec) +
    std::chrono::nanoseconds(pose.header.stamp.nanosec));
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      planning_frame_id_, pose.header.frame_id, time_point, tf2::durationFromSec(1.0));
    tf2::doTransform(pose, pose, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  return pose;
}

void HermitePathPlannerComponent::GoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!current_pose_) {
    RCLCPP_ERROR(this->get_logger(), "Current Pose does not recieved yet");
    return;
  }
  geometry_msgs::msg::PoseStamped goal_pose = TransformToPlanningFrame(*msg);
  geometry_msgs::msg::PoseStamped current_pose = TransformToPlanningFrame(*current_pose_);
  hermite_path_msgs::msg::HermitePathStamped path;
  path.path = generator_->generateHermitePath(current_pose.pose, goal_pose.pose);
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = planning_frame_id_;
  hermite_path_pub_->publish(path);
  marker_pub_->publish(generator_->generateMarker(path, 30));
}

void HermitePathPlannerComponent::CurrentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
}
}  // namespace hermite_path_planner

RCLCPP_COMPONENTS_REGISTER_NODE(hermite_path_planner::HermitePathPlannerComponent)
