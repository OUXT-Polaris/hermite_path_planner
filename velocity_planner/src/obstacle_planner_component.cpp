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
#include <hermite_path_planner/hermite_path_generator.hpp>
#include <velocity_planner/obstacle_planner_component.hpp>
#include <memory>
#include <set>
#include <string>

namespace velocity_planner
{
ObstaclePlannerComponent::ObstaclePlannerComponent(const rclcpp::NodeOptions & options)
: Node("obstacle_planner", "velocity_planner", options), buffer_(get_clock()), listener_(buffer_),
  viz_(get_name())
{
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  obstacle_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle/marker", 1);
  hermite_path_pub_ =
    this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>("~/hermite_path", 1);
  std::string update_path_topic;
  declare_parameter("update_path_topic", "/planner_concatenator/update");
  get_parameter("update_path_topic", update_path_topic);
  update_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>(
    update_path_topic, 1);
  std::string hermite_path_topic;
  declare_parameter("hermite_path_topic", "/hermite_path_planner/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    hermite_path_topic, 1,
    std::bind(&ObstaclePlannerComponent::hermitePathCallback, this, std::placeholders::_1));
  std::string obstacle_scan_topic;
  declare_parameter("obstacle_scan_topic", "/obstacle_scan");
  get_parameter("obstacle_scan_topic", obstacle_scan_topic);
  declare_parameter("robot_width", 1.5);
  get_parameter("robot_width", robot_width_);
  declare_parameter("stop_margin", 1.5);
  get_parameter("stop_margin", stop_margin_);
  declare_parameter("section_length", 0.5);
  get_parameter("section_length", section_length_);
  declare_parameter("max_linear_velocity", 0.5);
  get_parameter("max_linear_velocity", max_linear_velocity_);
  declare_parameter("max_deceleration", 0.5);
  get_parameter("max_deceleration", max_deceleration_);
  declare_parameter("t_upper_threashold", 1.2);
  get_parameter("t_upper_threashold", t_upper_threashold_);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    obstacle_scan_topic, 1,
    std::bind(&ObstaclePlannerComponent::scanCallback, this, std::placeholders::_1));
  std::string current_pose_topic;
  declare_parameter("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&ObstaclePlannerComponent::currentPoseCallback, this, std::placeholders::_1));
}

void ObstaclePlannerComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  current_pose_ = *data;
}

void ObstaclePlannerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{
  path_ = *data;
  target_obstacle_t_ = boost::none;
  path_ = addObstacleConstraints();
  if (path_) {
    marker_pub_->publish(viz_.generateDeleteMarker());
    marker_pub_->publish(viz_.generateMarker(path_.get(), color_names::makeColorMsg("lime", 1.0)));
    hermite_path_pub_->publish(path_.get());
  } else {
    marker_pub_->publish(viz_.generateDeleteMarker());
  }
}

geometry_msgs::msg::PoseStamped ObstaclePlannerComponent::TransformToMapFrame(
  geometry_msgs::msg::PoseStamped pose)
{
  if (pose.header.frame_id == "map") {
    return pose;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(pose.header.stamp.sec) +
    std::chrono::nanoseconds(pose.header.stamp.nanosec));

  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      buffer_.lookupTransform("map", pose.header.frame_id, time_point, tf2::durationFromSec(1.0));
    tf2::doTransform(pose, pose, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  return pose;
}

geometry_msgs::msg::PointStamped ObstaclePlannerComponent::TransformToMapFrame(
  geometry_msgs::msg::PointStamped point)
{
  if (point.header.frame_id == "map") {
    return point;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(point.header.stamp.sec) +
    std::chrono::nanoseconds(point.header.stamp.nanosec));
  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      buffer_.lookupTransform("map", point.header.frame_id, time_point, tf2::durationFromSec(1.0));
    tf2::doTransform(point, point, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  return point;
}

boost::optional<hermite_path_msgs::msg::HermitePathStamped>
ObstaclePlannerComponent::addObstacleConstraints()
{
  if (!path_ || !current_pose_ || !scan_) {
    return boost::none;
  }
  hermite_path_planner::HermitePathGenerator generator(0.0);
  geometry_msgs::msg::PoseStamped pose_transformed;
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(current_pose_->header.stamp.sec) +
    std::chrono::nanoseconds(current_pose_->header.stamp.nanosec));
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      path_.get().header.frame_id, current_pose_->header.frame_id, time_point,
      tf2::durationFromSec(1.0));
    tf2::doTransform(current_pose_.get(), pose_transformed, transform_stamped);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return boost::none;
  }
  auto current_t = generator.getNormalizedLongitudinalDistanceInFrenetCoordinate(
    path_->path, pose_transformed.pose.position);
  std::set<double> t_values;
  for (int i = 0; i < static_cast<int>(scan_->ranges.size()); i++) {
    if (scan_->range_max >= scan_->ranges[i] && scan_->ranges[i] >= scan_->range_min) {
      double theta = scan_->angle_min + scan_->angle_increment * static_cast<double>(i);
      geometry_msgs::msg::PointStamped p;
      p.point.x = scan_->ranges[i] * std::cos(theta);
      p.point.y = scan_->ranges[i] * std::sin(theta);
      p.point.z = 0.0;
      p.header = scan_->header;
      p = TransformToMapFrame(p);
      auto t_value = generator.getNormalizedLongitudinalDistanceInFrenetCoordinate(
        path_->path,
        p.point);
      if (t_value) {
        geometry_msgs::msg::Point nearest_point =
          generator.getPointOnHermitePath(path_->path, t_value.get());
        double lat_dist = std::sqrt(
          std::pow(nearest_point.x - p.point.x, 2) + std::pow(nearest_point.y - p.point.y, 2));
        if (std::fabs(lat_dist) < std::fabs(robot_width_) && t_value.get() > current_t.get()) {
          t_values.insert(t_value.get());
        }
      }
    }
  }
  if (t_values.size() != 0) {
    double t = *t_values.begin();
    if (t > t_upper_threashold_) {
      obstacle_marker_pub_->publish(viz_.generateDeleteMarker());
      return path_.get();
    }
    double length = generator.getLength(path_.get().path, 200);
    double target_t = t - (stop_margin_ / length);
    if (!target_obstacle_t_) {
      target_obstacle_t_ = target_t;
    } else {
      if (target_obstacle_t_.get() < target_t) {
        return boost::none;
      }
    }
    hermite_path_msgs::msg::HermitePathStamped path = path_.get();
    path.reference_velocity.clear();
    hermite_path_msgs::msg::ReferenceVelocity v;

    int i = 0;
    while (true) {
      double t = (length * target_t - (static_cast<double>(i) * section_length_)) / length;
      hermite_path_msgs::msg::ReferenceVelocity ref;
      ref.t = t;
      if (i == 0) {
        ref.stop_flag = true;
      }
      ref.linear_velocity = std::sqrt(2 * max_deceleration_ * (target_t - t));
      if (ref.linear_velocity > max_linear_velocity_) {
        break;
      }
      if (ref.t <= 0.0) {
        break;
      }
      ref.from_node = get_name();
      path.reference_velocity.push_back(ref);
      i++;
    }
    obstacle_marker_pub_->publish(viz_.generateDeleteMarker());
    obstacle_marker_pub_->publish(
      viz_.generateObstacleMarker(t, path_.get(), color_names::makeColorMsg("red", 0.8), 1.5));
    return path;
  } else {
    obstacle_marker_pub_->publish(viz_.generateDeleteMarker());
    return path_.get();
  }
  return boost::none;
}

void ObstaclePlannerComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  scan_ = *data;
  auto obstacle_path = addObstacleConstraints();
  if (obstacle_path) {
    if (obstacle_path.get().reference_velocity.size() != 0) {
      marker_pub_->publish(viz_.generateDeleteMarker());
      marker_pub_->publish(
        viz_.generateMarker(obstacle_path.get(), color_names::makeColorMsg("lime", 1.0)));
      update_pub_->publish(obstacle_path.get());
    }
  }
}
}  // namespace velocity_planner
