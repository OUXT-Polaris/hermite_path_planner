#include <velocity_planner/velocity_visualizer.h>
#include <color_names/color_names.h>

namespace velocity_planner
{
    VelocityVisualizer::VelocityVisualizer(std::string node_name)
    : generator_(0.0)
    {
        node_name_ = node_name;
    }

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generateDeleteMarker()
    {
        visualization_msgs::msg::MarkerArray ret;
        visualization_msgs::msg::Marker marker;
        marker.action = marker.DELETEALL;
        ret.markers.push_back(marker);
        return ret;
    }

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
        hermite_path_msgs::msg::HermitePathStamped path)
    {
        std_msgs::msg::ColorRGBA default_ref_color = color_names::makeColorMsg("lime",1.0);
        return generateMarker(path,default_ref_color);
    }

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generateMarker(
        hermite_path_msgs::msg::HermitePathStamped path,
        std_msgs::msg::ColorRGBA color_ref_velocity)
    {
        visualization_msgs::msg::MarkerArray marker;
        std::sort
        (
            path.reference_velocity.begin(),
            path.reference_velocity.end(),
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
            geometry_msgs::msg::Vector3 vec = generator_.getTangentVector(path.path,itr->t);
            double theta = std::atan2(vec.y,vec.x);
            geometry_msgs::msg::Vector3 orientation_vec;
            orientation_vec.x = 0.0;
            orientation_vec.y = 0.0;
            orientation_vec.z = theta;
            box_marker.action = box_marker.ADD;
            box_marker.pose.position = point;
            box_marker.pose.position.z = box_marker.pose.position.z + itr->linear_velocity*0.5;
            box_marker.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(orientation_vec);
            box_marker.scale.x = 0.1;
            box_marker.scale.y = 0.1;
            bool is_zero = (std::fabs(itr->linear_velocity) < DBL_EPSILON);
            if(is_zero)
            {
                box_marker.scale.z = 0.001;
            }
            else
            {
                box_marker.scale.z = itr->linear_velocity;
            }
            box_marker.color = color_ref_velocity;
            marker.markers.push_back(box_marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = path.header;
            text_marker.ns = "reference_velocity_text";
            text_marker.type = text_marker.TEXT_VIEW_FACING;
            text_marker.action = text_marker.ADD;
            text_marker.id = index;
            text_marker.pose.position = point;
            text_marker.pose.position.z = text_marker.pose.position.z + itr->linear_velocity*0.5 + 0.5;
            text_marker.scale.x = 0.2;
            text_marker.scale.y = 0.2;
            text_marker.scale.z = 0.2;
            text_marker.color = color_ref_velocity;
            char velocity_string[10];
            std::sprintf(velocity_string,"%.2f",itr->linear_velocity);
            std::string str(velocity_string,10);
            text_marker.text = str + "(m/s)";
            marker.markers.push_back(text_marker);
        }
        return marker;
    }
}