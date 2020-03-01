#include <velocity_planner/velocity_visualizer.h>

namespace velocity_planner
{
    VelocityVisualizer::VelocityVisualizer(std::string node_name)
    : generator_(0.0)
    {
        node_name_ = node_name;
    }

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
        hermite_path_msgs::msg::HermitePathStamped path)
    {
        std_msgs::msg::ColorRGBA default_ref_color;
        default_ref_color.r = 0.6;
        default_ref_color.g = 0.6;
        default_ref_color.b = 0.1;
        default_ref_color.a = 0.8;
        std_msgs::msg::ColorRGBA default_target_color;
        default_target_color.r = 0.8;
        default_target_color.g = 0.1;
        default_target_color.b = 0.1;
        default_target_color.a = 0.9;
        return generateMarker(path,default_ref_color,default_target_color);
    }

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
        hermite_path_msgs::msg::HermitePathStamped path,
        std_msgs::msg::ColorRGBA color_ref_velocity,
        std_msgs::msg::ColorRGBA color_target_velocity)
    {
        visualization_msgs::msg::MarkerArray marker;
        std::sort
        (
            path.reference_velocity.begin(),
            path.reference_velocity.end(),
            [](const auto& x, const auto& y)
                {return x.t < y.t;}
        );
        std::sort
        (
            path.target_velocity.begin(),
            path.target_velocity.end(),
            [](const auto& x, const auto& y)
                {return x.t < y.t;}
        );
        for(auto itr = path.reference_velocity.begin(); itr != path.reference_velocity.end(); itr++)
        {
            visualization_msgs::msg::Marker box_marker;
            box_marker.header = path.header;
            box_marker.ns = "reference_velocity";
            box_marker.type = box_marker.CUBE;
            std::size_t index = std::distance(path.reference_velocity.begin(), itr);
            box_marker.id = index;
            geometry_msgs::msg::Point point = generator_.getPointOnHermitePath(path.path,itr->t);
            box_marker.action = box_marker.ADD;
            box_marker.pose.position = point;
            box_marker.pose.position.z = box_marker.pose.position.z + itr->linear_velocity*0.5;
            box_marker.scale.x = 0.1;
            box_marker.scale.y = 0.1;
            box_marker.scale.z = itr->linear_velocity;
            box_marker.color = color_ref_velocity;
            marker.markers.push_back(box_marker);
        }
        for(auto itr = path.target_velocity.begin(); itr != path.target_velocity.end(); itr++)
        {
            visualization_msgs::msg::Marker box_marker;
            box_marker.header = path.header;
            box_marker.ns = "target_velocity";
            box_marker.type = box_marker.CUBE;
            std::size_t index = std::distance(path.target_velocity.begin(), itr);
            box_marker.id = index;
            geometry_msgs::msg::Point point = generator_.getPointOnHermitePath(path.path,itr->t);
            box_marker.action = box_marker.ADD;
            box_marker.pose.position = point;
            box_marker.pose.position.z = box_marker.pose.position.z + itr->linear_velocity*0.5;
            box_marker.scale.x = 0.3;
            box_marker.scale.y = 0.3;
            box_marker.scale.z = itr->linear_velocity;
            box_marker.color = color_target_velocity;
            marker.markers.push_back(box_marker);
        }
        return marker;
    }
}