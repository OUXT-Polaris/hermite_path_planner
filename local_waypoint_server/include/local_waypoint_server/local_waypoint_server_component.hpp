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

#ifndef LOCAL_WAYPOINT_SERVER__LOCAL_WAYPOINT_SERVER_COMPONENT_HPP_
#define LOCAL_WAYPOINT_SERVER__LOCAL_WAYPOINT_SERVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_EXPORT __attribute__((dllexport))
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_EXPORT __declspec(dllexport)
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_BUILDING_DLL
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC \
  LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_EXPORT
#else
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC \
  LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_IMPORT
#endif
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC_TYPE \
  LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_LOCAL
#else
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_LOCAL
#endif
#define LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace local_waypoint_server
{
class LocalWaypointServerComponent : public rclcpp::Node
{
public:
  LOCAL_WAYPOINT_SERVER_LOCAL_WAYPOINT_SERVER_COMPONENT_PUBLIC
  explicit LocalWaypointServerComponent(const rclcpp::NodeOptions & options);
  void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
  std::string goal_pose_topic_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_waypoint_pub_;
};
}  // namespace local_waypoint_server
#endif  // LOCAL_WAYPOINT_SERVER__LOCAL_WAYPOINT_SERVER_COMPONENT_HPP_
