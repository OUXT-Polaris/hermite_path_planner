#ifndef LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
#define LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_

#include <behaviortree_cpp_v3/action_node.h>

namespace local_waypoint_server
{
class ObstacleAvoid : public BT::SyncActionNode
{
public:
  ObstacleAvoid(const std::string & name);
  BT::NodeStatus tick() override;
};
}

#endif  // LOCAL_WAYPOINT_SERVER__OBSTACLE_AVOID_ACTION_HPP_
