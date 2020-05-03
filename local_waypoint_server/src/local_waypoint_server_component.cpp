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
  get_parameter("robot_width", robot_width_);
  generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width_);
  declare_parameter("planning_frame_id", "map");
  get_parameter("planning_frame_id", planning_frame_id_);
  declare_parameter("max_iterations", 10);
  get_parameter("max_iterations", max_iterations_);
  /**
   * Publishers
   */
  local_waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/local_waypoint",
      1);
  /**
   * Subscribers
   */
  std::string hermite_path_topic;
  declare_parameter("hermite_path_topic", "/hermite_path_planner/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    hermite_path_topic, 1,
    std::bind(&LocalWaypointServerComponent::hermitePathCallback, this, std::placeholders::_1));

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

void LocalWaypointServerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{
  current_path_ = *data;
}

void LocalWaypointServerComponent::GoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  updateLocalWaypoint();
}

bool LocalWaypointServerComponent::isSame(
  geometry_msgs::msg::PoseStamped pose0,
  geometry_msgs::msg::PoseStamped pose1)
{
  if (pose0.pose.position.x == pose1.pose.position.x) {
    if (pose0.pose.position.y == pose1.pose.position.y) {
      if (quaternion_operation::equals(pose0.pose.orientation, pose1.pose.orientation)) {
        return true;
      }
    }
  }
  return false;
}

void LocalWaypointServerComponent::updateLocalWaypoint()
{
  if (!scan_ || !current_pose_ || !goal_pose_) {
    return;
  }
  auto result = checkCollisionToCurrentPath();
  geometry_msgs::msg::PoseStamped current_target_pose;
  current_target_pose = goal_pose_.get();
  if (!previous_local_waypoint_) {
    previous_local_waypoint_ = current_target_pose;
    local_waypoint_pub_->publish(current_target_pose);
  } else {
    auto previous_local_waypoint = previous_local_waypoint_.get();
    if (!isSame(previous_local_waypoint, current_target_pose)) {
      previous_local_waypoint_ = current_target_pose;
      local_waypoint_pub_->publish(current_target_pose);
    }
  }
}

void LocalWaypointServerComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  scan_ = *data;
  scan_points_ = getPoints(*data);
  updateLocalWaypoint();
}

void LocalWaypointServerComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  current_pose_ = *data;
  updateLocalWaypoint();
}

boost::optional<double> LocalWaypointServerComponent::checkCollisionToCurrentPath(){
  if(!current_path_ || !current_pose_){
    return boost::none;
  }
  geometry_msgs::msg::PoseStamped pose_transformed = TransformToPlanningFrame(current_pose_.get());
  auto current_t = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(
    current_path_ ->path, pose_transformed.pose.position);
  std::set<double> t_values;
  for(auto points_itr = scan_points_.begin(); points_itr != scan_points_.end(); points_itr++){
    auto t_value = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(current_path_->path, *points_itr);
    if (t_value) {
      geometry_msgs::msg::Point nearest_point =
        generator_->getPointOnHermitePath(current_path_->path, t_value.get());
      double lat_dist = std::sqrt(
        std::pow(nearest_point.x - points_itr->x, 2) + std::pow(nearest_point.y - points_itr->y, 2));
      if (std::fabs(lat_dist) < std::fabs(robot_width_) && t_value.get() > current_t.get()) {
        t_values.insert(t_value.get());
      }
    }
  }
  if (t_values.size() != 0) {
    double t = *t_values.begin();
    return t;
  }
  return boost::none;
}

std::vector<geometry_msgs::msg::Point> LocalWaypointServerComponent::getPoints(
  sensor_msgs::msg::LaserScan scan)
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
    if (scan.range_max >= scan.ranges[i] && scan.ranges[i] >= scan.range_min) {
      double theta = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      geometry_msgs::msg::PointStamped p;
      p.point.x = scan.ranges[i] * std::cos(theta);
      p.point.y = scan.ranges[i] * std::sin(theta);
      p.point.z = 0.0;
      p.header = scan.header;
      p = TransformToPlanningFrame(p);
      ret.push_back(p.point);
    }
  }
  return ret;
}

geometry_msgs::msg::PointStamped LocalWaypointServerComponent::TransformToPlanningFrame(
  geometry_msgs::msg::PointStamped point)
{
  if (point.header.frame_id == planning_frame_id_) {
    return point;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(point.header.stamp.sec) +
    std::chrono::nanoseconds(point.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
    planning_frame_id_, point.header.frame_id, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(point, point, transform_stamped);
  return point;
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
    planning_frame_id_, pose.header.frame_id, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(pose, pose, transform_stamped);
  return pose;
}
}  // namespace local_waypoint_server
