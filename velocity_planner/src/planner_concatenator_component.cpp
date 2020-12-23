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

#include <velocity_planner/planner_concatenator_component.hpp>
#include <memory>
#include <algorithm>
#include <set>
#include <vector>
#include <list>
#include <map>
#include <string>
#include <utility>

namespace velocity_planner
{
PlannerConcatenatorComponent::PlannerConcatenatorComponent(const rclcpp::NodeOptions & options)
: Node("planner_concatenator", "velocity_planner", options), viz_(get_name())
{
  hermite_path_pub_ =
    this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>("~/hermite_path", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
  declare_parameter("num_input", 2);
  get_parameter("num_input", num_input_);
  assert(num_input_ >= 2 && num_input_ <= 8);
  for (int i = 0; i < num_input_; i++) {
    declare_parameter(
      "input_topic" + std::to_string(i), get_name() + std::string("/input") + std::to_string(i));
    get_parameter("input_topic" + std::to_string(i), input_topics_[i]);
    std::shared_ptr<HermitePathSubscriber> sub_ptr(
      new HermitePathSubscriber(this, input_topics_[i]));
    sub_ptrs_[i] = sub_ptr;
  }
  if (num_input_ == 2) {
    sync2_ =
      std::make_shared<message_filters::TimeSynchronizer<HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], 10);
    sync2_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback2, this, std::placeholders::_1,
        std::placeholders::_2));
  }
  if (num_input_ == 3) {
    sync3_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], 10);
    sync3_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback3, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
  }
  if (num_input_ == 4) {
    sync4_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], 10);
    sync4_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback4, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4));
  }
  if (num_input_ == 5) {
    sync5_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
          HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], 10);
    sync5_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback5, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
  }
  if (num_input_ == 6) {
    sync6_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
          HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5], 10);
    sync6_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback6, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,
        std::placeholders::_6));
  }
  if (num_input_ == 7) {
    sync7_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
          HermitePathStamped, HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5],
      *sub_ptrs_[6], 10);
    sync7_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback7, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
        std::placeholders::_7));
  }
  if (num_input_ == 8) {
    sync8_ = std::make_shared<message_filters::TimeSynchronizer<
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped,
          HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped>>(
      *sub_ptrs_[0], *sub_ptrs_[1], *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5],
      *sub_ptrs_[6], *sub_ptrs_[7], 10);
    sync8_->registerCallback(
      std::bind(
        &PlannerConcatenatorComponent::callback8, this, std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
        std::placeholders::_7, std::placeholders::_8));
  }
  update_sub_ = create_subscription<hermite_path_msgs::msg::HermitePathStamped>(
    "~/update", 1,
    std::bind(&PlannerConcatenatorComponent::updateCallback, this, std::placeholders::_1));
}

std::vector<hermite_path_msgs::msg::ReferenceVelocity>
PlannerConcatenatorComponent::filterReferenceVelocity(
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> data)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> ret;
  std::multimap<double, std::pair<std::string, double>> dict;
  std::list<double> keys;
  for (auto itr = data.begin(); itr != data.end(); itr++) {
    auto vel_from_pair = std::make_pair(itr->from_node, itr->linear_velocity);
    dict.emplace(itr->t, vel_from_pair);
    keys.emplace_back(itr->t);
  }
  keys.sort();
  keys.unique();
  for (auto key_itr = keys.begin(); key_itr != keys.end(); key_itr++) {
    if (*key_itr >= 0.0 && 1.0 >= *key_itr) {
      std::list<std::pair<std::string, double>> vels;
      auto p = dict.equal_range(*key_itr);
      for (auto it = p.first; it != p.second; ++it) {
        vels.emplace_back(it->second);
      }
      vels.sort();
      hermite_path_msgs::msg::ReferenceVelocity ref_vel;
      ref_vel.t = *key_itr;
      std::pair<std::string, double> vel = *vels.begin();
      ref_vel.linear_velocity = vel.second;
      ref_vel.from_node = vel.first;
      ret.push_back(ref_vel);
    }
  }
  return ret;
}

void PlannerConcatenatorComponent::callback2(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback3(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback4(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
  reference_vels.insert(reference_vels.end(), r3.begin(), r3.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback5(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
  const HermitePathStamped::ConstSharedPtr in4)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
  reference_vels.insert(reference_vels.end(), r3.begin(), r3.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
  reference_vels.insert(reference_vels.end(), r4.begin(), r4.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback6(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
  const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
  reference_vels.insert(reference_vels.end(), r3.begin(), r3.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
  reference_vels.insert(reference_vels.end(), r4.begin(), r4.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
  reference_vels.insert(reference_vels.end(), r5.begin(), r5.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback7(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
  const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
  const HermitePathStamped::ConstSharedPtr in6)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
  reference_vels.insert(reference_vels.end(), r3.begin(), r3.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
  reference_vels.insert(reference_vels.end(), r4.begin(), r4.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
  reference_vels.insert(reference_vels.end(), r5.begin(), r5.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r6 = in6->reference_velocity;
  reference_vels.insert(reference_vels.end(), r6.begin(), r6.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::callback8(
  const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
  const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
  const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
  const HermitePathStamped::ConstSharedPtr in6, const HermitePathStamped::ConstSharedPtr in7)
{
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
  reference_vels.insert(reference_vels.end(), r0.begin(), r0.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
  reference_vels.insert(reference_vels.end(), r1.begin(), r1.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
  reference_vels.insert(reference_vels.end(), r2.begin(), r2.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
  reference_vels.insert(reference_vels.end(), r3.begin(), r3.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
  reference_vels.insert(reference_vels.end(), r4.begin(), r4.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
  reference_vels.insert(reference_vels.end(), r5.begin(), r5.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r6 = in6->reference_velocity;
  reference_vels.insert(reference_vels.end(), r6.begin(), r6.end());
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> r7 = in7->reference_velocity;
  reference_vels.insert(reference_vels.end(), r7.begin(), r7.end());

  path_.header = in0->header;
  path_.path = in0->path;
  path_.reference_velocity = filterReferenceVelocity(reference_vels);
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}

void PlannerConcatenatorComponent::updateCallback(const HermitePathStamped::SharedPtr data)
{
  if (path_.path.ax != data->path.ax) {
    return;
  }
  if (path_.path.bx != data->path.bx) {
    return;
  }
  if (path_.path.cx != data->path.cx) {
    return;
  }
  if (path_.path.dx != data->path.dx) {
    return;
  }
  if (path_.path.ay != data->path.ay) {
    return;
  }
  if (path_.path.by != data->path.by) {
    return;
  }
  if (path_.path.cy != data->path.cy) {
    return;
  }
  if (path_.path.dy != data->path.dy) {
    return;
  }
  path_.header = data->header;
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> vel;
  std::set<std::string> update_targets;
  for (auto itr = data->reference_velocity.begin(); itr != data->reference_velocity.end(); itr++) {
    if (itr->t >= 0.0 && 1.0 >= itr->t) {
      update_targets.insert(itr->from_node);
    }
  }
  for (auto itr = path_.reference_velocity.begin(); itr != path_.reference_velocity.end(); itr++) {
    bool is_update_target = false;
    for (auto target_itr = update_targets.begin(); target_itr != update_targets.end();
      target_itr++)
    {
      if (*target_itr == itr->from_node) {
        is_update_target = true;
      }
    }
    if (!is_update_target) {
      vel.push_back(*itr);
    }
  }
  vel.insert(vel.end(), data->reference_velocity.begin(), data->reference_velocity.end());
  std::sort(vel.begin(), vel.end(), [](const auto & a, const auto & b) {return a.t < b.t;});
  std::vector<hermite_path_msgs::msg::ReferenceVelocity> filtered_vel;
  for (auto itr = vel.begin(); itr != vel.end(); itr++) {
    if (itr->t >= 0.0 && 1.0 >= itr->t) {
      filtered_vel.push_back(*itr);
    }
  }
  path_.reference_velocity = filtered_vel;
  hermite_path_pub_->publish(path_);
  marker_pub_->publish(viz_.generateDeleteMarker());
  marker_pub_->publish(viz_.generateMarker(path_));
}
}  // namespace velocity_planner
