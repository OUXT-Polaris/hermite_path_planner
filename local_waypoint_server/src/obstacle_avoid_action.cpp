#include <local_waypoint_server/obstacle_avoid_action.hpp>

namespace local_waypoint_server
{
ObstacleAvoid::ObstacleAvoid(const std::string & name)
: BT::SyncActionNode(name, {})
{
}

BT::NodeStatus ObstacleAvoid::tick()
{

}
}
