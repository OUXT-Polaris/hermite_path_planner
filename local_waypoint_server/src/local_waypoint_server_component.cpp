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
}

bool LocalWaypointServerComponent::checkCollision(geometry_msgs::msg::PoseStamped goal_pose, double& longitudal_distance)
{
  geometry_msgs::msg::PoseStamped current_goal_pose = TransformToPlanningFrame(goal_pose_.get());
  geometry_msgs::msg::PoseStamped current_pose = TransformToPlanningFrame(current_pose_.get());
  double goal_distance =
    std::sqrt(std::pow(goal_pose.pose.position.x - current_pose.pose.position.x, 2) +
      std::pow(goal_pose.pose.position.y - current_pose.pose.position.y, 2));
  hermite_path_msgs::msg::HermitePathStamped path;
  path.path = generator_->generateHermitePath(current_pose.pose, goal_pose.pose,
      goal_distance * 0.25, goal_distance * 0.75);
  path.header = goal_pose.header;
  std::vector<geometry_msgs::msg::Point> points = getPoints(scan_.get());
  std::vector<double> lateral_dists;
  std::vector<geometry_msgs::msg::Point> obstacle_points;
  for (auto itr = points.begin(); itr != points.end(); itr++) {
    auto dist = generator_->getLateralDistanceInFrenetCoordinate(path.path, *itr);
    if (dist) {
      lateral_dists.push_back(dist.get());
      obstacle_points.push_back(*itr);
    }
  }
  if (lateral_dists.size() == 0) {
    return false;
  }
  std::vector<double>::iterator min_itr = std::min_element(lateral_dists.begin(),
      lateral_dists.end());
  double min_lateral_dist = *min_itr;
  if (min_lateral_dist < robot_width_ * 0.5) {
    size_t index = std::distance(lateral_dists.begin(), min_itr);
    auto lon_dist = generator_->getLongitudinalDistanceInFrenetCoordinate(path.path, obstacle_points[index], 200);
    if(lon_dist){
      std::cout << lon_dist.get() << std::endl;
      longitudal_distance = lon_dist.get();
      return true;
    }
  }
  return false;
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
  geometry_msgs::msg::PoseStamped current_target_pose;
  current_target_pose = goal_pose_.get();
  if (!previous_local_waypoint_) {
    previous_local_waypoint_ = current_target_pose;
    local_waypoint_pub_->publish(current_target_pose);
  } else {
    auto previous_local_waypoint = previous_local_waypoint_.get();
    if (isSame(previous_local_waypoint, current_target_pose)) {
    }
  }
  double dist;
  if(checkCollision(current_target_pose,dist)){
    return;
  }
  std::cout << "collision not found" << std::endl;
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
    point.header.frame_id, planning_frame_id_, time_point, tf2::durationFromSec(1.0));
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
    pose.header.frame_id, planning_frame_id_, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(pose, pose, transform_stamped);
  return pose;
}
}  // namespace local_waypoint_server
