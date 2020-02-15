#include <hermite_path_planner/hermite_path_planner_component.h>

namespace hermite_path_planner
{
    HermitePathPlannerComponent::HermitePathPlannerComponent(const rclcpp::NodeOptions & options)
        : Node("navi_sim", options)
    {

    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hermite_path_planner::HermitePathPlannerComponent)