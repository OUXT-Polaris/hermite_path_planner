#include <velocity_planner/velocity_planner_component.h>

namespace velocity_planner
{
    VelocityPlannerComponent::VelocityPlannerComponent(const rclcpp::NodeOptions & options)
    : Node("velocity_planner", options),
        buffer_(get_clock()),
        listener_(buffer_)
    {

    }

    void VelocityPlannerComponent::hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
    {

    }
}