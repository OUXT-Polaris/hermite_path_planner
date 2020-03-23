#include <velocity_planner/obstacle_planner_component.h>
#include <hermite_path_planner/hermite_path_generator.h>
#include <color_names/color_names.h>

namespace velocity_planner
{
    ObstaclePlannerComponent::ObstaclePlannerComponent(const rclcpp::NodeOptions & options)
    : Node("obstacle_planner", options),
        buffer_(get_clock()),
        listener_(buffer_),
        viz_(get_name())
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>
            ("~/marker", 1);
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>
            ("~/obstacle/marker", 1);
        hermite_path_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>
            ("~/hermite_path", 1);
        std::string hermite_path_topic;
        declare_parameter("hermite_path_topic","/hermite_path_planner/hermite_path");
        get_parameter("hermite_path_topic",hermite_path_topic);
        hermite_path_sub_ = this->create_subscription<hermite_path_msgs::msg::HermitePathStamped>
            (hermite_path_topic, 1, std::bind(&ObstaclePlannerComponent::hermitePathCallback, this, std::placeholders::_1));
        std::string obstacle_scan_topic;
        declare_parameter("obstacle_scan_topic","/obstacle_scan");
        get_parameter("obstacle_scan_topic",obstacle_scan_topic);
        declare_parameter("robot_width",1.5);
        get_parameter("robot_width",robot_width_);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            (obstacle_scan_topic, 1, std::bind(&ObstaclePlannerComponent::scanCallback, this, std::placeholders::_1));
    }

    void ObstaclePlannerComponent::hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data)
    {
        path_ = *data;
        marker_pub_->publish(viz_.generateDeleteMarker());
        marker_pub_->publish(viz_.generateMarker(path_.get(),color_names::makeColorMsg("lime",1.0)));
        hermite_path_pub_->publish(path_.get());
    }

    geometry_msgs::msg::PointStamped ObstaclePlannerComponent::TransformToMapFrame(geometry_msgs::msg::PointStamped point)
    {
        if(point.header.frame_id == "map")
        {
            return point;
        }
        tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(point.header.stamp.sec) +
            std::chrono::nanoseconds(point.header.stamp.nanosec));
        geometry_msgs::msg::TransformStamped transform_stamped = 
            buffer_.lookupTransform("map", point.header.frame_id,
                time_point, tf2::durationFromSec(1.0));
        tf2::doTransform(point, point, transform_stamped);
        return point;
    }

    void ObstaclePlannerComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
    {
        if(!path_)
        {
            return;
        }
        hermite_path_planner::HermitePathGenerator generator(0.0);
        std::set<double> t_values;
        for(int i=0; i<(int)data->ranges.size(); i++)
        {
            if(data->range_max >= data->ranges[i]  && data->ranges[i] >= data->range_min)
            {
                double theta = data->angle_min + data->angle_increment * (double)i;
                geometry_msgs::msg::PointStamped p;
                p.point.x = data->ranges[i] * std::cos(theta);
                p.point.y = data->ranges[i] * std::sin(theta);
                p.point.z = 0.0;
                p.header = data->header;
                p = TransformToMapFrame(p);
                auto t_value = generator.getLongitudinalDistanceInFrenetCoordinate(path_->path,p.point);
                if(t_value)
                {
                    geometry_msgs::msg::Point nearest_point = generator.getPointOnHermitePath(path_->path,t_value.get());
                    double lat_dist = std::sqrt(std::pow(nearest_point.x-p.point.x,2)+std::pow(nearest_point.y-p.point.y,2));
                    if(std::fabs(lat_dist) < std::fabs(robot_width_))
                    {
                        t_values.insert(t_value.get());
                    }
                }
            }
        }
        if(t_values.size() != 0)
        {
            double t = *t_values.begin();
            obstacle_marker_pub_->publish(viz_.generateDeleteMarker());
            obstacle_marker_pub_->publish(viz_.generateObstacleMarker(t,path_.get(),color_names::makeColorMsg("red",0.8)));
        }
        else
        {
            obstacle_marker_pub_->publish(viz_.generateDeleteMarker());
        }
    }
}