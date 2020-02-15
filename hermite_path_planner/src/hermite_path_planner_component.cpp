#include <hermite_path_planner/hermite_path_planner_component.h>

namespace hermite_path_planner
{
    HermitePathPlannerComponent::HermitePathPlannerComponent(const rclcpp::NodeOptions & options)
        : Node("hermite_path_planner", options),
            buffer_(get_clock()),
            listener_(buffer_)
    {
        declare_parameter("planning_frame_id","map");
        get_parameter("planning_frame_id",planning_frame_id_);
        declare_parameter("goal_pose_topic","/move_base_simple/goal");
        get_parameter("goal_pose_topic",goal_pose_topic_);
        declare_parameter("current_pose_topic","/current_pose");
        get_parameter("current_pose_topic",current_pose_topic_);
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (goal_pose_topic_, 1, std::bind(&HermitePathPlannerComponent::GoalPoseCallback, this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (current_pose_topic_, 1, std::bind(&HermitePathPlannerComponent::GoalPoseCallback, this, std::placeholders::_1));
    }

    geometry_msgs::msg::PoseStamped HermitePathPlannerComponent::TransformToPlanningFrame(geometry_msgs::msg::PoseStamped pose)
    {
        if(pose.header.frame_id == planning_frame_id_)
        {
            return pose;
        }
        tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(pose.header.stamp.sec) +
            std::chrono::nanoseconds(pose.header.stamp.nanosec));
        geometry_msgs::msg::TransformStamped transform_stamped = 
            buffer_.lookupTransform(pose.header.frame_id, planning_frame_id_, 
                time_point, tf2::durationFromSec(1.0));
        tf2::doTransform(pose, pose, transform_stamped);
        return pose;
    }

    void HermitePathPlannerComponent::GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(!current_pose_)
        {
            return;
        }
        geometry_msgs::msg::PoseStamped goal_pose = TransformToPlanningFrame(*msg);
        geometry_msgs::msg::PoseStamped current_pose = TransformToPlanningFrame(*current_pose_);
        hermite_path_msgs::msg::HermitePathStamped path;
        path.path = generateHermitePath(current_pose.pose,goal_pose.pose);
        path.header.stamp = msg->header.stamp;
        path.header.frame_id = planning_frame_id_;
        return;
    }

    void HermitePathPlannerComponent::CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        return;
    }

    hermite_path_msgs::msg::HermitePath HermitePathPlannerComponent::generateHermitePath(geometry_msgs::msg::Pose start,geometry_msgs::msg::Pose goal)
    {
        hermite_path_msgs::msg::HermitePath path;
        geometry_msgs::msg::Vector3 start_vector = getVectorFromPose(start,10.0);
        geometry_msgs::msg::Vector3 goal_vector = getVectorFromPose(goal,10.0);
        path.ax = 2*start.position.x - 2*goal.position.x + start_vector.x + goal_vector.x;
        path.ay = 2*start.position.y - 2*goal.position.y + start_vector.y + goal_vector.y;
        path.bx = -3*start.position.x + 3*goal.position.x - 2*start_vector.x - goal_vector.x;
        path.by = -3*start.position.y + 3*goal.position.y - 2*start_vector.y - goal_vector.y;
        path.cx = start_vector.x;
        path.cy = start_vector.y;
        path.dx = start.position.x;
        path.dy = start.position.y;
        return path;
    }

    geometry_msgs::msg::Vector3 HermitePathPlannerComponent::getVectorFromPose(geometry_msgs::msg::Pose pose,double magnitude)
    {
        geometry_msgs::msg::Vector3 vector = quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
        vector.z = 0;
        double ratio = magnitude/std::sqrt(vector.x*vector.x + vector.y*vector.y);
        vector.x = vector.x * ratio;
        vector.y = vector.y * ratio;
        return vector;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hermite_path_planner::HermitePathPlannerComponent)