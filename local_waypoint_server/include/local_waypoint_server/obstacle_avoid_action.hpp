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

#ifndef LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
#define LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_

#include <string>
#include <memory>
#include "behavior_tree_action_builder/action_node.hpp"
#include "behavior_tree_action_builder/register_nodes.hpp"

namespace local_waypoint_server
{
class ObstacleAvoidAction : public behavior_tree_action_builder::ActionNode
{
public:
  explicit ObstacleAvoidAction(const std::string & name, const BT::NodeConfiguration & config);
  BT::NodeStatus tick() override;
};
}

#endif  // LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
