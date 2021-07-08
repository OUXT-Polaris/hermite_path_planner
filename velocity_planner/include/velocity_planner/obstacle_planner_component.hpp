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

#ifndef VELOCITY_PLANNER__OBSTACLE_PLANNER_COMPONENT_HPP_
#define VELOCITY_PLANNER__OBSTACLE_PLANNER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __attribute__((dllexport))
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_BUILDING_DLL
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT
#else
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT
#endif
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC_TYPE \
  VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL
#else
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL
#endif
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <velocity_planner/velocity_visualizer.hpp>
#include <boost/optional.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace velocity_planner
{
class ObstaclePlannerComponent : public rclcpp::Node
{
public:
  VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
  explicit ObstaclePlannerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
  void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
  rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
  rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr update_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data);
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::msg::PointStamped TransformToMapFrame(geometry_msgs::msg::PointStamped point);
  geometry_msgs::msg::PoseStamped TransformToMapFrame(geometry_msgs::msg::PoseStamped pose);
  double robot_width_;
  VelocityVisualizer viz_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  boost::optional<sensor_msgs::msg::LaserScan> scan_;
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> addObstacleConstraints();
  double stop_margin_;
  double section_length_;
  double max_linear_velocity_;
  double max_deceleration_;
  double t_upper_threashold_;
  boost::optional<double> target_obstacle_t_;
};
}  // namespace velocity_planner

#endif  // VELOCITY_PLANNER__OBSTACLE_PLANNER_COMPONENT_HPP_
