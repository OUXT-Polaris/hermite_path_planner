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

#ifndef VELOCITY_PLANNER__VELOCITY_PLANNER_COMPONENT_HPP_
#define VELOCITY_PLANNER__VELOCITY_PLANNER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __attribute__((dllexport))
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_BUILDING_DLL
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT
#else
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT
#endif
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC_TYPE \
  VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL
#else
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL
#endif
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/optional.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <hermite_path_planner/hermite_path_generator.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <velocity_planner/velocity_visualizer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace velocity_planner
{
class VelocityPlannerComponent : public rclcpp::Node
{
public:
  VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
  explicit VelocityPlannerComponent(const rclcpp::NodeOptions & options);

private:
  void callback(
    const hermite_path_msgs::msg::HermitePathStamped::SharedPtr curve_path,
    const hermite_path_msgs::msg::HermitePathStamped::SharedPtr obstacle_path);
  rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
  void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_stamped_sub_;
  boost::optional<geometry_msgs::msg::Twist> current_twist_;
  boost::optional<geometry_msgs::msg::TwistStamped> current_twist_stamped_;
  void currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  void currentTwistStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void updatePath();
  void checkCurrentPath();
  bool checkTopics();
  bool twist_stamped_;
  std::mutex mtx_;
  double robot_width;
  std::shared_ptr<hermite_path_planner::HermitePathGenerator> generator_;
  double maximum_accerelation_ = 0.3;
  double minimum_accerelation_ = -0.1;
  double max_linear_velocity_ = 1.0;
  double velocity_resoluation_ = 0.2;
  bool use_velocity_graph_;
  VelocityVisualizer viz_;
};
}  // namespace velocity_planner

#endif  //  VELOCITY_PLANNER__VELOCITY_PLANNER_COMPONENT_HPP_
