#include <velocity_planner/stop_planner_component.h>

namespace velocity_planner
{
    StopPlannerComponent::StopPlannerComponent(const rclcpp::NodeOptions & options)
    : Node("stop_planner", options), generator_(0.0), viz_(get_name())
    {
        hermite_path_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>
            ("~/hermite_path", 1);
        std::string hermite_path_topic;
        declare_parameter("hermite_path_topic","/hermite_path_planner/hermite_path");
        get_parameter("hermite_path_topic",hermite_path_topic);
        hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>
            (hermite_path_topic, 1, std::bind(&StopPlannerComponent::hermitePathCallback, this, std::placeholders::_1));
    }

    void StopPlannerComponent::hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
    {
        path_ = *data;
        hermite_path_pub_->publish(path_.get());
    }
}