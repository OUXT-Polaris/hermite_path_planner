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

    visualization_msgs::msg::MarkerArray VelocityVisualizer::generatePolygonMarker(
        hermite_path_msgs::msg::HermitePathStamped path)
        {
            visualization_msgs::msg::MarkerArray ret;
            int path_length = path.reference_velocity.size();
            for(int i=0; i<(path_length-1); i++)
            {
                visualization_msgs::msg::Marker polygon;
                polygon.header = path.header;
                polygon.ns = "polygon";
                polygon.id = i;
                polygon.type = polygon.TRIANGLE_LIST;
                polygon.action = polygon.ADD;
                polygon.scale.x = 1.0;
                polygon.scale.y = 1.0;
                polygon.scale.z = 1.0;
                double t0 = path.reference_velocity[i].t;
                double t1 = path.reference_velocity[i+1].t;
                double v = (path.reference_velocity[i].linear_velocity + path.reference_velocity[i+1].linear_velocity)/2.0;
                geometry_msgs::msg::Vector3 vec0 = generator_.getNormalVector(path.path,t0);
                geometry_msgs::msg::Vector3 vec1 = generator_.getNormalVector(path.path,t1);
                double theta0 = std::atan2(vec0.y,vec0.x);
                double theta1 = std::atan2(vec1.y,vec1.x);
                geometry_msgs::msg::Point p0 = generator_.getPointOnHermitePath(path.path,t0);
                geometry_msgs::msg::Point p1 = generator_.getPointOnHermitePath(path.path,t1);
                geometry_msgs::msg::Point p0l;
                p0l.x = p0.x - 0.5*std::cos(theta0);
                p0l.y = p0.y - 0.5*std::sin(theta0);
                geometry_msgs::msg::Point p0r;
                p0r.x = p0.x + 0.5*std::cos(theta0);
                p0r.y = p0.y + 0.5*std::sin(theta0);
                geometry_msgs::msg::Point p1l;
                p1l.x = p1.x - 0.5*std::cos(theta1);
                p1l.y = p1.y - 0.5*std::sin(theta1);
                geometry_msgs::msg::Point p1r;
                p1r.x = p1.x + 0.5*std::cos(theta1);
                p1r.y = p1.y + 0.5*std::sin(theta1);
                polygon.points.push_back(p0l);
                polygon.points.push_back(p0r);
                polygon.points.push_back(p1l);
                polygon.points.push_back(p1l);
                polygon.points.push_back(p0r);
                polygon.points.push_back(p1r);
                std_msgs::msg::ColorRGBA color;
                double ratio = std::fabs(v)/0.5;
                color.r = 1.0-ratio;
                color.g = ratio;
                color.b = 0.3;
                color.a = 1.0;
                polygon.color = color;
                ret.markers.push_back(polygon);
            }
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