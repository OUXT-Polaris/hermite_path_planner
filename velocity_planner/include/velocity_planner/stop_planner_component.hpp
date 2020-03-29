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

#ifndef VELOCITY_PLANNER__STOP_PLANNER_COMPONENT_HPP_
#define VELOCITY_PLANNER__STOP_PLANNER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_EXPORT __attribute__((dllexport))
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_BUILDING_DLL
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_EXPORT
#else
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_IMPORT
#endif
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC_TYPE \
  VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_LOCAL
#else
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_LOCAL
#endif
#define VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <hermite_path_planner/hermite_path_generator.hpp>
#include <velocity_planner/velocity_visualizer.hpp>
#include <boost/optional.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace velocity_planner
{
class StopPlannerComponent : public rclcpp::Node
{
public:
  VELOCITY_PLANNER_STOP_PLANNER_COMPONENT_PUBLIC
  explicit StopPlannerComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
  rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
  void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  hermite_path_planner::HermitePathGenerator generator_;
  VelocityVisualizer viz_;
  double max_deceleration_;
  double max_linear_velocity_;
  double section_length_;
};
}  // namespace velocity_planner

#endif  // VELOCITY_PLANNER__STOP_PLANNER_COMPONENT_HPP_
