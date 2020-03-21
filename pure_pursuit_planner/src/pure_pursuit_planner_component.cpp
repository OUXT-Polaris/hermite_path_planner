#include <pure_pursuit_planner/pure_pursuit_planner_component.h>
#include <color_names/color_names.h>

namespace pure_pursuit_planner
{
    PurePursuitPlannerComponent::PurePursuitPlannerComponent(const rclcpp::NodeOptions & options)
    : Node("pure_pursuit_planner", options),
        buffer_(get_clock()),
        listener_(buffer_)
    {
        declare_parameter("robot_width",3.0);
        double robot_width;
        get_parameter("robot_width",robot_width);
        generator_ = std::make_shared<hermite_path_planner::HermitePathGenerator>(robot_width);
        declare_parameter("minimum_lookahead_distance",2.0);
        get_parameter("minimum_lookahead_distance",minimum_lookahead_distance_);
        lookahead_distance_ = minimum_lookahead_distance_;
        declare_parameter("lookahead_ratio",1.5);
        get_parameter("lookahead_ratio",lookahead_ratio_);

        std::string current_twist_topic;
        declare_parameter("current_twist_topic","current_twist");
        get_parameter("current_twist_topic",current_twist_topic);
        current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
            (current_twist_topic, 1, std::bind(&PurePursuitPlannerComponent::currentTwistCallback, this, std::placeholders::_1));

        std::string current_pose_topic;
        declare_parameter("current_pose_topic","current_pose");
        get_parameter("current_pose_topic",current_pose_topic);
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
            (current_pose_topic, 1, std::bind(&PurePursuitPlannerComponent::currentPoseCallback, this, std::placeholders::_1));

        std::string hermite_path_topic;
        declare_parameter("hermite_path_topic","/velocity_planner/hermite_path");
        get_parameter("hermite_path_topic",hermite_path_topic);
        hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>
            (hermite_path_topic, 1, std::bind(&PurePursuitPlannerComponent::hermitePathCallback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker",1);
        target_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("target_twist",1);
    }

    void PurePursuitPlannerComponent::currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
    {
        current_twist_ = *data;
        if(lookahead_ratio_*data->linear.x < minimum_lookahead_distance_)
        {
            lookahead_distance_ = minimum_lookahead_distance_;
        }
        else
        {
            lookahead_distance_ = lookahead_ratio_*data->linear.x;
        }
    }

    void PurePursuitPlannerComponent::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
    {
        current_pose_ = *data;
        if(!path_)
        {
            return;
        }
        geometry_msgs::msg::PoseStamped pose;
        if(path_->header.frame_id != current_pose_->header.frame_id)
        {
            tf2::TimePoint time_point = tf2::TimePoint(
                std::chrono::seconds(current_pose_->header.stamp.sec) +
                std::chrono::nanoseconds(current_pose_->header.stamp.nanosec));
            geometry_msgs::msg::TransformStamped transform_stamped = 
                buffer_.lookupTransform(current_pose_->header.frame_id, path_->header.frame_id,
                    time_point, tf2::durationFromSec(1.0));
            tf2::doTransform(*current_pose_, pose, transform_stamped);
        }
        else
        {
            pose = current_pose_.get();
        }
        visualization_msgs::msg::MarkerArray marker;
        boost::optional<double> t = generator_->checkFirstCollisionWithCircle(path_->path,pose.pose.position,lookahead_distance_);
        if(t)
        {
            geometry_msgs::msg::Point target_position = generator_->getPointOnHermitePath(path_->path,t.get());
            // draw target marker
            visualization_msgs::msg::Marker target_marker;
            target_marker.header = pose.header;
            target_marker.ns = "target";
            target_marker.id = 0;
            target_marker.action = visualization_msgs::msg::Marker::ADD;
            target_marker.type = visualization_msgs::msg::Marker::SPHERE;
            target_marker.pose.position = target_position;
            target_marker.pose.orientation.x = 0.0f;
            target_marker.pose.orientation.y = 0.0f;
            target_marker.pose.orientation.z = 0.0f;
            target_marker.pose.orientation.w = 1.0f;
            target_marker.scale.x = 0.3;
            target_marker.scale.y = 0.3;
            target_marker.scale.z = 0.3;
            target_marker.color = color_names::makeColorMsg("yellow",1.0);
            marker.markers.push_back(target_marker);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "failed to find target point");
        }
        // draw search circle
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header = pose.header;
        circle_marker.ns = "circle";
        circle_marker.id = 0;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        circle_marker.scale.x = 0.1;
        circle_marker.color = color_names::makeColorMsg("chocolate",1.0);
        constexpr int circle_marker_resolution = 200;
        for(int i=0;i<(circle_marker_resolution+1); i++)
        {
            double theta = 2*M_PI/(double)circle_marker_resolution * i;
            geometry_msgs::msg::Point p;
            p.x = pose.pose.position.x + lookahead_distance_ * std::cos(theta);
            p.y = pose.pose.position.y + lookahead_distance_ * std::sin(theta);
            circle_marker.points.push_back(p);
        }
        marker.markers.push_back(circle_marker);
        marker_pub_->publish(marker);
    }

    boost::optional<geometry_msgs::msg::Twist> PurePursuitPlannerComponent::getCurrentTwist()
    {
        if(!current_pose_)
        {
            return boost::none;
        }
        geometry_msgs::msg::Twist ret;
        boost::optional<double> t = generator_->checkFirstCollisionWithCircle(path_->path,current_pose_->pose.position,lookahead_distance_);
        if(!t)
        {
            return boost::none;
        }
        geometry_msgs::msg::Point target_position = generator_->getPointOnHermitePath(path_->path,t.get());
        geometry_msgs::msg::Vector3 rpy = quaternion_operation::convertQuaternionToEulerAngle(current_pose_->pose.orientation);
        double theta = rpy.z;
        double alpha = std::atan2(target_position.y - current_pose_->pose.position.y, target_position.x - current_pose_->pose.position.x) - theta;
        double r = std::sqrt(std::pow(target_position.x - current_pose_->pose.position.x, 2) + std::pow(target_position.y - current_pose_->pose.position.y, 2)) / (2*std::sin(alpha));
        double length = r * alpha;
        //double omega = 
        return ret;
    }

    void PurePursuitPlannerComponent::hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
    {
        path_ = *data;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit_planner::PurePursuitPlannerComponent)