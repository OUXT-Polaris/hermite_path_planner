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
#include <memory>
#include <vector>
#include <string>
#include <set>

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
  declare_parameter("num_candidates", 20);
  get_parameter("num_candidates", num_candidates_);
  declare_parameter("sampling_interval", 0.5);
  get_parameter("sampling_interval", sampling_interval_);
  declare_parameter("sampling_offset", 2.0);
  get_parameter("sampling_offset", sampling_offset_);
  declare_parameter("margin", 0.5);
  get_parameter("margin", margin_);
  /**
   * Publishers
   */
  local_waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/local_waypoint",
    1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  marker_no_collision_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/marker/no_collision", 1);
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
  updateLocalWaypoint();
}

void LocalWaypointServerComponent::GoalPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  replaned_goalpose_ = boost::none;
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

std::vector<geometry_msgs::msg::Pose> LocalWaypointServerComponent::getLocalWaypointCandidates(
  double obstacle_t)
{
  std::vector<geometry_msgs::msg::Pose> ret;
  if (!current_path_) {
    return ret;
  }
  geometry_msgs::msg::Pose p;
  p.position = generator_->getPointOnHermitePath(current_path_->path, obstacle_t);
  geometry_msgs::msg::Vector3 tangent_vec = generator_->getTangentVector(
    current_path_->path,
    obstacle_t);
  double theta = std::atan2(tangent_vec.y, tangent_vec.x);
  geometry_msgs::msg::Vector3 orientation_vec;
  orientation_vec.x = 0.0;
  orientation_vec.y = 0.0;
  orientation_vec.z = theta;
  p.orientation = quaternion_operation::convertEulerAngleToQuaternion(orientation_vec);
  geometry_msgs::msg::Vector3 n = generator_->getNormalVector(current_path_->path, obstacle_t);
  double norm = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  n.x = n.x / norm;
  n.y = n.y / norm;
  n.z = n.z / norm;
  for (int i = 0; (i * 2) < num_candidates_; i++) {
    geometry_msgs::msg::Pose p0 = p;
    p0.position.x = p0.position.x +
      (sampling_offset_ + static_cast<double>(i + 1) * sampling_interval_) * n.x;
    p0.position.y = p0.position.y +
      (sampling_offset_ + static_cast<double>(i + 1) * sampling_interval_) * n.y;
    ret.push_back(p0);
    geometry_msgs::msg::Pose p1 = p;
    p1.position.x = p1.position.x -
      (sampling_offset_ + static_cast<double>(i + 1) * sampling_interval_) * n.x;
    p1.position.y = p1.position.y -
      (sampling_offset_ + static_cast<double>(i + 1) * sampling_interval_) * n.y;
    ret.push_back(p1);
  }
  return ret;
}

void LocalWaypointServerComponent::updateLocalWaypoint()
{
  if (!scan_ || !current_pose_ || !goal_pose_) {
    return;
  }
  auto result = checkCollisionToCurrentPath();
  if (result) {
    if (result.get() <= 1.0) {
      std::vector<geometry_msgs::msg::Pose> candidates = getLocalWaypointCandidates(result.get());
      replaned_goalpose_ = evaluateCandidates(candidates);
      if (replaned_goalpose_) {
        geometry_msgs::msg::PoseStamped p;
        p.pose = replaned_goalpose_.get();
        p.header.frame_id = planning_frame_id_;
        p.header.stamp = get_clock()->now();
        previous_local_waypoint_ = p;
        local_waypoint_pub_->publish(p);
        return;
      }
    }
  }
  if (!replaned_goalpose_) {
    geometry_msgs::msg::PoseStamped current_goal_pose;
    current_goal_pose = goal_pose_.get();
    if (!previous_local_waypoint_) {
      previous_local_waypoint_ = current_goal_pose;
      local_waypoint_pub_->publish(current_goal_pose);
    } else {
      auto previous_local_waypoint = previous_local_waypoint_.get();
      if (!isSame(previous_local_waypoint, current_goal_pose)) {
        previous_local_waypoint_ = current_goal_pose;
        local_waypoint_pub_->publish(current_goal_pose);
      }
    }
  } else {
    double dist_to_replaned_goal = std::sqrt(
      std::pow(
        current_pose_->pose.position.x - replaned_goalpose_->position.x, 2) + std::pow(
        current_pose_->pose.position.y - replaned_goalpose_->position.y, 2));
    if (dist_to_replaned_goal < 1.3) {
      replaned_goalpose_ = boost::none;
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

boost::optional<geometry_msgs::msg::Pose> LocalWaypointServerComponent::evaluateCandidates(
  std::vector<geometry_msgs::msg::Pose> candidates)
{
  if (!current_pose_ || !goal_pose_) {
    return boost::none;
  }
  rclcpp::Time now = get_clock()->now();
  std::vector<hermite_path_msgs::msg::HermitePathStamped> path_lists;
  std::vector<hermite_path_msgs::msg::HermitePathStamped> non_collision_path_lists;
  geometry_msgs::msg::PoseStamped start = TransformToPlanningFrame(current_pose_.get());
  std::vector<geometry_msgs::msg::Pose> non_collision_goal_list;
  for (auto pose_itr = candidates.begin(); pose_itr != candidates.end(); pose_itr++) {
    auto path = generator_->generateHermitePath(start.pose, *pose_itr);
    hermite_path_msgs::msg::HermitePathStamped stamped_path;
    stamped_path.header.frame_id = planning_frame_id_;
    stamped_path.header.stamp = now;
    stamped_path.path = path;
    path_lists.push_back(stamped_path);
    auto obstacle_distance = checkCollisionToPath(path);
    auto after_path = generator_->generateHermitePath(*pose_itr, goal_pose_.get().pose);
    auto obstacle_distance_in_after_path = checkCollisionToPath(after_path);
    if (!obstacle_distance && !obstacle_distance_in_after_path) {
      non_collision_goal_list.push_back(*pose_itr);
      non_collision_path_lists.push_back(stamped_path);
    }
  }
  marker_pub_->publish(generator_->generateDeleteMarker());
  marker_pub_->publish(generator_->generateMarker(path_lists, 200));
  marker_no_collision_pub_->publish(generator_->generateDeleteMarker());
  marker_no_collision_pub_->publish(generator_->generateMarker(non_collision_path_lists, 200));
  if (non_collision_goal_list.size() == 0) {
    RCLCPP_ERROR(get_logger(), "stacked");
    return boost::none;
  }
  std::vector<double> distance_to_goal;
  for (auto itr = non_collision_goal_list.begin(); itr != non_collision_goal_list.end(); itr++) {
    double dist =
      std::sqrt(
      std::pow(
        goal_pose_->pose.position.x - itr->position.x,
        2) + std::pow(goal_pose_->pose.position.y - itr->position.y, 2));
    distance_to_goal.push_back(dist);
  }
  std::vector<double>::iterator min_itr = std::min_element(
    distance_to_goal.begin(), distance_to_goal.end());
  size_t min_index = std::distance(distance_to_goal.begin(), min_itr);
  return non_collision_goal_list[min_index];
}

boost::optional<double> LocalWaypointServerComponent::checkCollisionToPath(
  hermite_path_msgs::msg::HermitePath path)
{
  if (!current_pose_) {
    return boost::none;
  }
  geometry_msgs::msg::PoseStamped pose_transformed = TransformToPlanningFrame(current_pose_.get());
  std::set<double> t_values;
  for (auto points_itr = scan_points_.begin(); points_itr != scan_points_.end(); points_itr++) {
    auto t_value = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(
      path, *points_itr);
    if (t_value) {
      if (t_value.get() <= 1.3 && t_value.get() >= 0.0) {
        if (t_value.get() >= 1.0) {
          t_value.get() = 1.0;
        }
        geometry_msgs::msg::Point nearest_point =
          generator_->getPointOnHermitePath(path, t_value.get());
        double lat_dist = std::sqrt(
          std::pow(
            nearest_point.x - points_itr->x,
            2) + std::pow(nearest_point.y - points_itr->y, 2));
        if (lat_dist < std::fabs(robot_width_ * 0.5 + margin_)) {
          t_values.insert(t_value.get());
        }
      }
    }
  }
  if (t_values.size() != 0) {
    double t = *t_values.begin();
    return t;
  }
  return boost::none;
}

boost::optional<double> LocalWaypointServerComponent::checkCollisionToCurrentPath()
{
  if (!current_path_ || !current_pose_) {
    return boost::none;
  }
  geometry_msgs::msg::PoseStamped pose_transformed = TransformToPlanningFrame(current_pose_.get());
  auto current_t = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(
    current_path_->path, pose_transformed.pose.position);
  std::set<double> t_values;
  for (auto points_itr = scan_points_.begin(); points_itr != scan_points_.end(); points_itr++) {
    auto t_value = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(
      current_path_->path, *points_itr);
    if (t_value) {
      geometry_msgs::msg::Point nearest_point =
        generator_->getPointOnHermitePath(current_path_->path, t_value.get());
      double lat_dist = std::sqrt(
        std::pow(
          nearest_point.x - points_itr->x,
          2) + std::pow(nearest_point.y - points_itr->y, 2));
      if (std::fabs(lat_dist) < std::fabs(robot_width_ * 0.5 + margin_) &&
        t_value.get() > current_t.get())
      {
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
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      planning_frame_id_, point.header.frame_id, time_point, tf2::durationFromSec(1.0));
    tf2::doTransform(point, point, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
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
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      planning_frame_id_, pose.header.frame_id, time_point, tf2::durationFromSec(1.0));
    tf2::doTransform(pose, pose, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  return pose;
}
}  // namespace local_waypoint_server
