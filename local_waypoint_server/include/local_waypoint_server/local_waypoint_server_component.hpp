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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hermite_path_planner/hermite_path_generator.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <boost/optional.hpp>
#include <string>
#include <memory>
#include <vector>

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
  boost::optional<geometry_msgs::msg::PoseStamped> goal_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_waypoint_pub_;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  boost::optional<sensor_msgs::msg::LaserScan> scan_;
  std::vector<geometry_msgs::msg::Point> scan_points_;
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  void updateLocalWaypoint();
  std::shared_ptr<hermite_path_planner::HermitePathGenerator> generator_;
  std::string planning_frame_id_;
  geometry_msgs::msg::PoseStamped TransformToPlanningFrame(geometry_msgs::msg::PoseStamped pose);
  geometry_msgs::msg::PointStamped TransformToPlanningFrame(geometry_msgs::msg::PointStamped pose);
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::vector<geometry_msgs::msg::Point> getPoints(sensor_msgs::msg::LaserScan scan);
  bool checkCollision(geometry_msgs::msg::PoseStamped goal_pose, double & longitudinal_distance);
  double robot_width_;
  boost::optional<geometry_msgs::msg::PoseStamped> previous_local_waypoint_;
  bool isSame(geometry_msgs::msg::PoseStamped pose0, geometry_msgs::msg::PoseStamped pose1);
  rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
  void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
  boost::optional<hermite_path_msgs::msg::HermitePathStamped> current_path_;
  boost::optional<double> checkCollisionToCurrentPath();
  boost::optional<double> checkCollisionToPath(hermite_path_msgs::msg::HermitePath path);
  std::vector<geometry_msgs::msg::Pose> getLocalWaypointCandidates(double obstacle_t);
  boost::optional<geometry_msgs::msg::Pose> evaluateCandidates(
    std::vector<geometry_msgs::msg::Pose> candidates);
  int num_candidates_;
  double sampling_interval_;
  double sampling_offset_;
  double margin_;
  boost::optional<geometry_msgs::msg::Pose> replaned_goalpose_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_no_collision_pub_;
};
}  // namespace local_waypoint_server
#endif  // LOCAL_WAYPOINT_SERVER__LOCAL_WAYPOINT_SERVER_COMPONENT_HPP_
