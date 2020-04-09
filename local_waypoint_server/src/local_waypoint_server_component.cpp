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

#include <local_waypoint_server/local_waypoint_server_component.hpp>
#include <local_waypoint_server/actions/stopped_at_obstacle_action.hpp>
#include <local_waypoint_server/actions/stopping_at_obstacle_action.hpp>
#include <local_waypoint_server/actions/follow_path_action.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

namespace local_waypoint_server
{
LocalWaypointServerComponent::LocalWaypointServerComponent(const rclcpp::NodeOptions & options)
: Node("local_waypoint_server", options)
{
  declare_parameter("behavior_tree_xml_path", "");
  get_parameter("behavior_tree_xml_path", xml_path_);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FollowPathAction>("FollowPathAction");
  factory.registerNodeType<StoppingAtObstacleAction>("StoppingAtObstacleAction");
  factory.registerNodeType<StoppedAtObstacleAction>("StoppedAtObstacleAction");
  // BT::Tree tree;
  try {
    auto tree = factory.createTreeFromFile(xml_path_);
    printTreeRecursively(tree.root_node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
  }
  // BT::PublisherZMQ publisher_zmq(tree);
}
}  // namespace local_waypoint_server
