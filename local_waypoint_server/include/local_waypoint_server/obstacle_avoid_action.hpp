#ifndef LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
#define LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

namespace local_waypoint_server
{
class ObstacleAvoidAction : public BT::SyncActionNode
{
public:
  ObstacleAvoidAction(const std::string & name);
  BT::NodeStatus tick() override;
};
}

#endif  // LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
