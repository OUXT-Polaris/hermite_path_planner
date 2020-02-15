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
        hermite_path_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>(get_name()+std::string("/hermite_path"),1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(get_name()+std::string("/marker"),1);
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (goal_pose_topic_, 1, std::bind(&HermitePathPlannerComponent::GoalPoseCallback, this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (current_pose_topic_, 1, std::bind(&HermitePathPlannerComponent::CurrentPoseCallback, this, std::placeholders::_1));
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
            RCLCPP_ERROR(this->get_logger(), "Current Pose does not recieved yet");
            return;
        }
        geometry_msgs::msg::PoseStamped goal_pose = TransformToPlanningFrame(*msg);
        geometry_msgs::msg::PoseStamped current_pose = TransformToPlanningFrame(*current_pose_);
        hermite_path_msgs::msg::HermitePathStamped path;
        path.path = generateHermitePath(current_pose.pose,goal_pose.pose);
        path.header.stamp = msg->header.stamp;
        path.header.frame_id = planning_frame_id_;
        hermite_path_pub_->publish(path);
        marker_pub_->publish(generateMarker(path,200));
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
        geometry_msgs::msg::Vector3 start_vector = getVectorFromPose(start,30.0);
        geometry_msgs::msg::Vector3 goal_vector = getVectorFromPose(goal,30.0);
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

    #include <sstream>

    geometry_msgs::msg::Vector3 HermitePathPlannerComponent::getVectorFromPose(geometry_msgs::msg::Pose pose,double magnitude)
    {
        geometry_msgs::msg::Vector3 dir = quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
        geometry_msgs::msg::Vector3 vector;
        vector.x = magnitude * std::cos(dir.z);
        vector.y = magnitude * std::sin(dir.z);
        vector.z = 0;
        return vector;
    }

    geometry_msgs::msg::Point HermitePathPlannerComponent::getPointOnHermitePath(hermite_path_msgs::msg::HermitePath path,double t)
    {
        geometry_msgs::msg::Point p;
        p.z = 0;
        p.x = path.ax*std::pow(t,3) + path.bx*std::pow(t,2) + path.cx*t + path.dx;
        p.y = path.ay*std::pow(t,3) + path.by*std::pow(t,2) + path.cy*t + path.dy;
        return p;
    }

    visualization_msgs::msg::MarkerArray HermitePathPlannerComponent::generateMarker(hermite_path_msgs::msg::HermitePathStamped path,int resolution)
    {
        visualization_msgs::msg::MarkerArray marker;
        double step_size = 1.0/(double)resolution;

        // Setup Color
        std_msgs::msg::ColorRGBA color_center_line;
        color_center_line.r = 1.0;
        color_center_line.g = 0.0;
        color_center_line.b = 0.0;
        color_center_line.a = 1.0;
        
        // Setup Center Line Marker
        visualization_msgs::msg::Marker center_line;
        center_line.header = path.header;
        center_line.ns = "center_line";
        center_line.id = 0;
        center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        center_line.action = visualization_msgs::msg::Marker::ADD;
        center_line.frame_locked = false;
        center_line.scale.x = 0.1;
        for(int i=0;i<(resolution+1);i++)
        {
            double t = step_size * (double)i;
            center_line.points.push_back(getPointOnHermitePath(path.path,t));
            center_line.colors.push_back(color_center_line);
        }
        marker.markers.push_back(center_line);
        return marker;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hermite_path_planner::HermitePathPlannerComponent)