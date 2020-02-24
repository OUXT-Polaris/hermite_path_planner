#include <velocity_planner/velocity_planner_component.h>

namespace velocity_planner
{
    VelocityPlannerComponent::VelocityPlannerComponent(const rclcpp::NodeOptions & options)
    : Node("velocity_planner", options),
        buffer_(get_clock()),
        listener_(buffer_)
    {
        std::string current_twist_topic;
        declare_parameter("current_twist_topic","current_twist");
        get_parameter("current_twist_topic",current_twist_topic);
        current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
            (current_twist_topic, 1, std::bind(&VelocityPlannerComponent::currentTwistCallback, this, std::placeholders::_1));

        std::string hermite_path_topic;
        declare_parameter("hermite_path_topic","/hermite_path_planner/hermite_path");
        get_parameter("hermite_path_topic",hermite_path_topic);
        hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>
            (hermite_path_topic, 1, std::bind(&VelocityPlannerComponent::hermitePathCallback, this, std::placeholders::_1));

        std::string current_pose_topic;
        declare_parameter("current_pose_topic","current_pose");
        get_parameter("current_pose_topic",current_pose_topic);
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (current_pose_topic, 1, std::bind(&VelocityPlannerComponent::currentPoseCallback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker",1);

        double robot_width;
        get_parameter("robot_width",robot_width);
        generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width);

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(100ms, std::bind(&VelocityPlannerComponent::updatePath, this));
    }

    void VelocityPlannerComponent::updatePath()
    {
        mtx_.lock();
        if(path_ == boost::none)
        {
            //RCLCPP_INFO(get_logger(),"path was not calculated");
            mtx_.unlock();
            return;
        }
        if(current_twist_ == boost::none)
        {
            //RCLCPP_INFO(get_logger(),"current_twist did not published");
            mtx_.unlock();
            return;
        }
        if(current_pose_ == boost::none)
        {
            //RCLCPP_INFO(get_logger(),"current_pose did not published");
            mtx_.unlock();
            return;
        }
        if(path_->target_velocity.size() == 0)
        {
            hermite_path_msgs::msg::ReferenceVelocity vel;
            vel.t = 1.0;
            vel.linear_velocity = 0.0;
            path_->target_velocity.push_back(vel);
        }
        mtx_.unlock();
    }

    void VelocityPlannerComponent::hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
    {
        path_ = *data;
        updatePath();
    }

    void VelocityPlannerComponent::currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
    {
        current_twist_ = *data;
    }

    void VelocityPlannerComponent::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
    {
        current_pose_ = *data;
    }
}