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

#include <local_waypoint_server/local_waypoint_server_component.hpp>

namespace local_waypoint_server
{
LocalWaypointServerComponent::LocalWaypointServerComponent(const rclcpp::NodeOptions & options)
: Node("local_waypoint_server", options), buffer_(get_clock()), listener_(buffer_)
{
  declare_parameter("robot_width", 3.0);
  double robot_width;
  get_parameter("robot_width", robot_width);
  generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width);
  declare_parameter("planning_frame_id", "map");
  get_parameter("planning_frame_id", planning_frame_id_);
  /**
   * Publishers
   */
  local_waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/local_waypoint",
      1);
  /**
   * Subscribers
   */
  declare_parameter("goal_pose_topic", "/move_base_simple/goal");
  get_parameter("goal_pose_topic", goal_pose_topic_);
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_pose_topic_, 1,
    std::bind(&LocalWaypointServerComponent::GoalPoseCallback, this, std::placeholders::_1));
  std::string current_pose_topic;
  declare_parameter("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&LocalWaypointServerComponent::currentPoseCallback, this, std::placeholders::_1));
  std::string obstacle_scan_topic;
  declare_parameter("obstacle_scan_topic", "/obstacle_scan");
  get_parameter("obstacle_scan_topic", obstacle_scan_topic);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    obstacle_scan_topic, 1,
    std::bind(&LocalWaypointServerComponent::scanCallback, this, std::placeholders::_1));
}

void LocalWaypointServerComponent::GoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  updateLocalWaypoint();
  local_waypoint_pub_->publish(*msg);
}

void LocalWaypointServerComponent::updateLocalWaypoint()
{
  if (!scan_ || !current_pose_ || !goal_pose_) {
    return;
  }
}

void LocalWaypointServerComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  scan_ = *data;
  updateLocalWaypoint();
}

void LocalWaypointServerComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  current_pose_ = *data;
  updateLocalWaypoint();
}

geometry_msgs::msg::PoseStamped LocalWaypointServerComponent::TransformToPlanningFrame(
  geometry_msgs::msg::PoseStamped pose)
{
  if (pose.header.frame_id == planning_frame_id_) {
    return pose;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(pose.header.stamp.sec) +
    std::chrono::nanoseconds(pose.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
    pose.header.frame_id, planning_frame_id_, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(pose, pose, transform_stamped);
  return pose;
}
}  // namespace local_waypoint_server
