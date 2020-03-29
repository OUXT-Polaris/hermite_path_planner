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

#ifndef VELOCITY_PLANNER__PLANNER_CONCATENATOR_COMPONENT_HPP_
#define VELOCITY_PLANNER__PLANNER_CONCATENATOR_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT __attribute__((dllexport))
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT __declspec(dllexport)
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_BUILDING_DLL
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT
#else
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC \
  VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT
#endif
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC_TYPE \
  VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL
#else
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL
#endif
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <velocity_planner/velocity_visualizer.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <memory>
#include <vector>

namespace velocity_planner
{
typedef hermite_path_msgs::msg::HermitePathStamped HermitePathStamped;
typedef message_filters::Subscriber<HermitePathStamped> HermitePathSubscriber;
typedef message_filters::sync_policies::ExactTime<
    HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
    HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped>
  SyncPolicy;

class PlannerConcatenatorComponent : public rclcpp::Node
{
public:
  VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
  explicit PlannerConcatenatorComponent(const rclcpp::NodeOptions & options);

private:
  VelocityVisualizer viz_;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> filterReferenceVelocity(
    std::vector<hermite_path_msgs::msg::ReferenceVelocity> data);
  std::array<std::shared_ptr<HermitePathSubscriber>, 8> sub_ptrs_;
  rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
  int num_input_;
  std::array<std::string, 8> input_topics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<message_filters::TimeSynchronizer<HermitePathStamped, HermitePathStamped>>
  sync2_;
  void callback2(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1);

  std::shared_ptr<
    message_filters::TimeSynchronizer<HermitePathStamped, HermitePathStamped, HermitePathStamped>>
  sync3_;
  void callback3(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2);

  std::shared_ptr<message_filters::TimeSynchronizer<
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped>>
  sync4_;
  void callback4(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3);

  std::shared_ptr<message_filters::TimeSynchronizer<
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
      HermitePathStamped>>
  sync5_;
  void callback5(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
    const HermitePathStamped::ConstSharedPtr in4);

  std::shared_ptr<message_filters::TimeSynchronizer<
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
      HermitePathStamped, HermitePathStamped>>
  sync6_;
  void callback6(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
    const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5);

  std::shared_ptr<message_filters::TimeSynchronizer<
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
      HermitePathStamped, HermitePathStamped, HermitePathStamped>>
  sync7_;
  void callback7(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
    const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
    const HermitePathStamped::ConstSharedPtr in6);

  std::shared_ptr<message_filters::TimeSynchronizer<
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped>>
  sync8_;
  void callback8(
    const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
    const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
    const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
    const HermitePathStamped::ConstSharedPtr in6, const HermitePathStamped::ConstSharedPtr in7);
  hermite_path_msgs::msg::HermitePathStamped path_;
  void updateCallback(const HermitePathStamped::SharedPtr data);
  rclcpp::Subscription<HermitePathStamped>::SharedPtr update_sub_;
};
}  // namespace velocity_planner

#endif  // VELOCITY_PLANNER__PLANNER_CONCATENATOR_COMPONENT_HPP_
