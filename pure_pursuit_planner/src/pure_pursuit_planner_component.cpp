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

#include <pure_pursuit_planner/pure_pursuit_planner_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <color_names/color_names.hpp>
#include <string>
#include <memory>

namespace pure_pursuit_planner
{
PurePursuitPlannerComponent::PurePursuitPlannerComponent(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_planner", options), buffer_(get_clock()), listener_(buffer_)
{
  declare_parameter("robot_width", 3.0);
  double robot_width;
  get_parameter("robot_width", robot_width);
  generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width);
  declare_parameter("minimum_lookahead_distance", 2.0);
  get_parameter("minimum_lookahead_distance", minimum_lookahead_distance_);
  lookahead_distance_ = minimum_lookahead_distance_;
  declare_parameter("lookahead_ratio", 1.5);
  get_parameter("lookahead_ratio", lookahead_ratio_);

  std::string current_twist_topic;
  declare_parameter("current_twist_topic", "current_twist");
  get_parameter("current_twist_topic", current_twist_topic);
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    current_twist_topic, 1,
    std::bind(&PurePursuitPlannerComponent::currentTwistCallback, this, std::placeholders::_1));

  std::string current_pose_topic;
  declare_parameter("current_pose_topic", "current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&PurePursuitPlannerComponent::currentPoseCallback, this, std::placeholders::_1));

  std::string hermite_path_topic;
  declare_parameter("hermite_path_topic", "/velocity_planner/velocity_planner/hermite_path");
  get_parameter("hermite_path_topic", hermite_path_topic);
  hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    hermite_path_topic, 1,
    std::bind(&PurePursuitPlannerComponent::hermitePathCallback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  target_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("target_twist", 1);
}

void PurePursuitPlannerComponent::currentTwistCallback(
  const geometry_msgs::msg::Twist::SharedPtr data)
{
  current_twist_ = *data;
  if (lookahead_ratio_ * data->linear.x < minimum_lookahead_distance_) {
    lookahead_distance_ = minimum_lookahead_distance_;
  } else {
    lookahead_distance_ = lookahead_ratio_ * data->linear.x;
  }
}

visualization_msgs::msg::MarkerArray PurePursuitPlannerComponent::generateMarker(
  boost::optional<geometry_msgs::msg::Twist> twist)
{
  visualization_msgs::msg::MarkerArray marker;
  // draw text marker
  if (current_pose_transformed_) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = current_pose_transformed_->header;
    text_marker.ns = "target_twist";
    text_marker.id = 0;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.pose.position = current_pose_transformed_->pose.position;
    text_marker.pose.position.z = text_marker.pose.position.z + 2.5;
    text_marker.pose.orientation.x = 0.0f;
    text_marker.pose.orientation.y = 0.0f;
    text_marker.pose.orientation.z = 0.0f;
    text_marker.pose.orientation.w = 1.0f;
    text_marker.scale.x = 0.4;
    text_marker.scale.y = 0.4;
    text_marker.scale.z = 0.4;
    if (twist) {
      text_marker.text = "linear:" + std::to_string(twist->linear.x) + "(m/s)\nangular:" +
        std::to_string(twist->angular.z) + "(rad/s)";
      double ratio = std::fabs(twist->linear.x) / 0.8;
      text_marker.color = color_names::fromHsv(0.6 * ratio, 1.0, 1.0, 1.0);
    } else {
      text_marker.text = "linear : invalid\n angular : invalid";
      text_marker.color = color_names::makeColorMsg("white", 1.0);
    }
    marker.markers.push_back(text_marker);
  }
  if (target_position_ && current_pose_transformed_) {
    // draw target marker
    visualization_msgs::msg::Marker target_marker;
    target_marker.header = current_pose_transformed_->header;
    target_marker.ns = "target";
    target_marker.id = 0;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.pose.position = target_position_.get();
    target_marker.pose.orientation.x = 0.0f;
    target_marker.pose.orientation.y = 0.0f;
    target_marker.pose.orientation.z = 0.0f;
    target_marker.pose.orientation.w = 1.0f;
    target_marker.scale.x = 0.3;
    target_marker.scale.y = 0.3;
    target_marker.scale.z = 0.3;
    target_marker.color = color_names::makeColorMsg("yellow", 1.0);
    marker.markers.push_back(target_marker);
  } else {
    // delete target marker
    visualization_msgs::msg::Marker target_marker;
    target_marker.header = current_pose_transformed_->header;
    target_marker.ns = "target";
    target_marker.id = 0;
    target_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.markers.push_back(target_marker);
    RCLCPP_ERROR(get_logger(), "failed to find target point");
  }
  if (current_t_) {
    geometry_msgs::msg::Point current_position =
      generator_->getPointOnHermitePath(path_->path, current_t_.get());
    // draw current marker
    visualization_msgs::msg::Marker current_marker;
    current_marker.header = current_pose_transformed_->header;
    current_marker.ns = "current";
    current_marker.id = 0;
    current_marker.action = visualization_msgs::msg::Marker::ADD;
    current_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_marker.pose.position = current_position;
    current_marker.pose.orientation.x = 0.0f;
    current_marker.pose.orientation.y = 0.0f;
    current_marker.pose.orientation.z = 0.0f;
    current_marker.pose.orientation.w = 1.0f;
    current_marker.scale.x = 0.3;
    current_marker.scale.y = 0.3;
    current_marker.scale.z = 0.3;
    current_marker.color = color_names::makeColorMsg("green", 1.0);
    marker.markers.push_back(current_marker);
  } else {
    // delete current marker
    // draw target marker
    visualization_msgs::msg::Marker current_marker;
    current_marker.header = current_pose_transformed_->header;
    current_marker.ns = "current";
    current_marker.id = 0;
    current_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.markers.push_back(current_marker);
    RCLCPP_ERROR(get_logger(), "failed to current position in path");
  }

  // draw search circle
  visualization_msgs::msg::Marker circle_marker;
  circle_marker.header = current_pose_transformed_->header;
  circle_marker.ns = "circle";
  circle_marker.id = 0;
  circle_marker.action = visualization_msgs::msg::Marker::ADD;
  circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle_marker.scale.x = 0.1;
  circle_marker.color = color_names::makeColorMsg("chocolate", 1.0);
  constexpr int circle_marker_resolution = 200;
  for (int i = 0; i < (circle_marker_resolution + 1); i++) {
    double theta = 2 * M_PI / static_cast<double>(circle_marker_resolution) * i;
    geometry_msgs::msg::Point p;
    p.x = current_pose_transformed_->pose.position.x + lookahead_distance_ * std::cos(theta);
    p.y = current_pose_transformed_->pose.position.y + lookahead_distance_ * std::sin(theta);
    circle_marker.points.push_back(p);
  }
  marker.markers.push_back(circle_marker);
  return marker;
}

void PurePursuitPlannerComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  geometry_msgs::msg::Twist twist_cmd;
  current_pose_ = *data;
  if (!path_) {
    target_twist_pub_->publish(twist_cmd);
    return;
  }
  if (path_->header.frame_id != current_pose_->header.frame_id) {
    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(current_pose_->header.stamp.sec) +
      std::chrono::nanoseconds(current_pose_->header.stamp.nanosec));
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
        current_pose_->header.frame_id, path_->header.frame_id, time_point,
        tf2::durationFromSec(1.0));
      tf2::doTransform(*current_pose_, current_pose_transformed_.get(), transform_stamped);
    } catch (tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), ex.what());
      return;
    }
  } else {
    current_pose_transformed_ = current_pose_.get();
  }
  target_t_ = generator_->checkFirstCollisionWithCircle(
    path_->path, current_pose_transformed_->pose.position, lookahead_distance_);
  current_t_ = generator_->getNormalizedLongitudinalDistanceInFrenetCoordinate(
    path_->path, current_pose_->pose.position);
  if (!target_t_ && !current_t_) {
    twist_cmd.linear.x = 0;
    twist_cmd.linear.y = 0;
    twist_cmd.linear.z = 0;
    twist_cmd.angular.x = 0;
    twist_cmd.angular.y = 0;
    twist_cmd.angular.z = 0;
    target_twist_pub_->publish(twist_cmd);
    return;
  }
  // overwrite invalid result
  if (target_t_ && current_t_) {
    if (target_t_.get() < current_t_.get()) {
      target_t_ = boost::none;
    }
  }
  if (!target_t_ && current_t_) {
    target_t_ = 1.0;
  }
  auto twist = getTargetTwist(target_t_.get());
  if (twist) {
    twist_cmd = twist.get();
  } else {
    RCLCPP_ERROR(get_logger(), "failed to get twist cmd");
  }
  marker_pub_->publish(generateMarker(twist));
  target_twist_pub_->publish(twist_cmd);
}

boost::optional<geometry_msgs::msg::Twist> PurePursuitPlannerComponent::getTargetTwist(
  double target_t)
{
  if (!current_pose_) {
    RCLCPP_WARN(get_logger(), "current pose does not subscribe yet");
    return boost::none;
  }
  if (!path_) {
    RCLCPP_WARN(get_logger(), "path does not subscribe yet");
    return boost::none;
  }
  if (!current_t_) {
    RCLCPP_WARN(get_logger(), "t value on the current path path does not subscribe yet");
    return boost::none;
  }
  geometry_msgs::msg::Point target_position;
  geometry_msgs::msg::Twist ret;
  if (0.0 < target_t && target_t < 1.0) {
    target_position = generator_->getPointOnHermitePath(path_->path, target_t);
  } else if (target_t < 0.0) {
    geometry_msgs::msg::Vector3 vec = generator_->getTangentVector(path_->path, 0.0);
    double norm = std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    vec.x = vec.x / norm;
    vec.y = vec.y / norm;
    vec.z = vec.z / norm;
    geometry_msgs::msg::Point p = generator_->getPointOnHermitePath(path_->path, 0.0);
    double x = p.x - current_pose_->pose.position.x;
    double y = p.y - current_pose_->pose.position.y;
    double a = vec.x * vec.x + vec.y * vec.y;
    double b = 2 * x * vec.x + 2 * y * vec.y;
    double c = x * x + y * y - lookahead_distance_ * lookahead_distance_;
    if (b * b - 4 * a * c < 0.0) {
      target_position_ = boost::none;
      RCLCPP_WARN(
        get_logger(), "target position does not calculate because of b * b - 4 * a * c < 0.0");
      return boost::none;
    }
    double s0 = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);
    double s1 = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
    if (s0 < 0.0) {
      if (s1 > 0.0) {
        target_position.x = p.x + s1 * vec.x;
        target_position.y = p.y + s1 * vec.y;
        target_position.z = p.z + s1 * vec.z;
      } else {
        target_position_ = boost::none;
        return boost::none;
      }
    } else {
      target_position.x = p.x + s0 * vec.x;
      target_position.y = p.y + s0 * vec.y;
      target_position.z = p.z + s0 * vec.z;
    }
  } else {
    geometry_msgs::msg::Vector3 vec = generator_->getTangentVector(path_->path, 1.0);
    double norm = std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    vec.x = vec.x / norm;
    vec.y = vec.y / norm;
    vec.z = vec.z / norm;
    geometry_msgs::msg::Point p = generator_->getPointOnHermitePath(path_->path, 1.0);
    double x = p.x - current_pose_->pose.position.x;
    double y = p.y - current_pose_->pose.position.y;
    double a = vec.x * vec.x + vec.y * vec.y;
    double b = 2 * x * vec.x + 2 * y * vec.y;
    double c = x * x + y * y - lookahead_distance_ * lookahead_distance_;
    if ((b * b - 4 * a * c) < 0.0) {
      target_position_ = boost::none;
      RCLCPP_WARN(
        get_logger(), "target position does not calculate because of b * b - 4 * a * c < 0.0");
      return boost::none;
    }
    double s0 = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);
    double s1 = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
    if (s0 < 0.0) {
      target_position.x = p.x + s1 * vec.x;
      target_position.y = p.y + s1 * vec.y;
      target_position.z = p.z + s1 * vec.z;
    } else {
      target_position.x = p.x + s0 * vec.x;
      target_position.y = p.y + s0 * vec.y;
      target_position.z = p.z + s0 * vec.z;
    }
  }
  target_position_ = target_position;
  geometry_msgs::msg::Vector3 diff_rpy;
  diff_rpy.z = std::atan2(
    target_position.y - current_pose_->pose.position.y,
    target_position.x - current_pose_->pose.position.x);
  auto diff_quat = quaternion_operation::convertEulerAngleToQuaternion(diff_rpy);
  auto rot_quat = quaternion_operation::getRotation(current_pose_->pose.orientation, diff_quat);
  double alpha = quaternion_operation::convertQuaternionToEulerAngle(rot_quat).z;
  double r = std::sqrt(
    std::pow(target_position.x - current_pose_->pose.position.x, 2) +
    std::pow(target_position.y - current_pose_->pose.position.y, 2)) /
    (2 * std::sin(alpha));
  double length = r * alpha;
  double linear_velocity = generator_->getReferenceVelocity(path_.get(), current_t_.get());
  double omega = 2 * linear_velocity * std::sin(alpha) / length;
  ret.linear.x = linear_velocity;
  ret.linear.y = 0.0;
  ret.linear.z = 0.0;
  ret.angular.x = 0.0;
  ret.angular.y = 0.0;
  ret.angular.z = omega;
  return ret;
}

void PurePursuitPlannerComponent::hermitePathCallback(
  const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
{
  path_ = *data;
}
}  // namespace pure_pursuit_planner

RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit_planner::PurePursuitPlannerComponent)
