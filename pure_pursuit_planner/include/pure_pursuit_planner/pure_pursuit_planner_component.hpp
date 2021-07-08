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

#ifndef PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_
#define PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT __attribute__((dllexport))
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_BUILDING_DLL
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC \
  PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT
#else
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC \
  PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT
#endif
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC_TYPE \
  PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL
#else
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL
#endif
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <hermite_path_planner/hermite_path_generator.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <boost/optional.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <memory>

namespace pure_pursuit_planner
{
class PurePursuitPlannerComponent : public rclcpp::Node
{
public:
  PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
  explicit PurePursuitPlannerComponent(const rclcpp::NodeOptions & options);

private:
  boost::optional<geometry_msgs::msg::Twist> getTargetTwist(double target_t);
  void currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
  void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
  rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
  boost::optional<geometry_msgs::msg::Twist> current_twist_;
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_transformed_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_twist_pub_;
  std::shared_ptr<hermite_path_planner::HermitePathGenerator> generator_;
  double lookahead_distance_;
  double minimum_lookahead_distance_;
  double lookahead_ratio_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
  boost::optional<double> current_t_, target_t_;
  boost::optional<geometry_msgs::msg::Point> target_position_;
  visualization_msgs::msg::MarkerArray generateMarker(
    boost::optional<geometry_msgs::msg::Twist> twist);
};
}  // namespace pure_pursuit_planner

#endif  // PURE_PURSUIT_PLANNER__PURE_PURSUIT_PLANNER_COMPONENT_HPP_
