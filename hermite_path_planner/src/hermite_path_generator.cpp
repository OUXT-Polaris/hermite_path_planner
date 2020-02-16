#include <hermite_path_planner/hermite_path_generator.h>

namespace hermite_path_planner
{
    HermitePathGenerator::HermitePathGenerator(double robot_length,double robot_width)
    {
        robot_length_ = robot_length;
        robot_width_ = robot_width;
    }

    hermite_path_msgs::msg::HermitePath HermitePathGenerator::generateHermitePath(geometry_msgs::msg::Pose start,geometry_msgs::msg::Pose goal)
    {
        hermite_path_msgs::msg::HermitePath path;
        geometry_msgs::msg::Vector3 start_vector = getVectorFromPose(start,100.0);
        geometry_msgs::msg::Vector3 goal_vector = getVectorFromPose(goal,100.0);
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

    std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getPointsOnHermitePath(hermite_path_msgs::msg::HermitePath path,int resolution)
    {
        std::vector<geometry_msgs::msg::Point> p;
        double step_size = 1.0/(double)resolution;
        for(int i=0;i<(resolution+1);i++)
        {
            double t = step_size * (double)i;
            p.push_back(getPointOnHermitePath(path,t));
        }
        return p;
    }

    std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getLeftBounds(hermite_path_msgs::msg::HermitePath path,int resolution)
    {
        std::vector<geometry_msgs::msg::Point> points;
        double step_size = 1.0/(double)resolution;
        for(int i=0;i<(resolution+1);i++)
        {
            double t = step_size * (double)i;
            geometry_msgs::msg::Vector3 vec = getNormalVector(path,t);
            double theta = std::atan2(vec.y,vec.x);
            geometry_msgs::msg::Point p = getPointOnHermitePath(path,t);
            geometry_msgs::msg::Point point;
            point.x = p.x - robot_width_*std::cos(theta);
            point.y = p.y - robot_width_*std::sin(theta);
            points.push_back(point);
        }
        return points;
    }

    geometry_msgs::msg::Point HermitePathGenerator::getPointOnHermitePath(hermite_path_msgs::msg::HermitePath path,double t)
    {
        geometry_msgs::msg::Point p;
        p.z = 0;
        p.x = path.ax*std::pow(t,3) + path.bx*std::pow(t,2) + path.cx*t + path.dx;
        p.y = path.ay*std::pow(t,3) + path.by*std::pow(t,2) + path.cy*t + path.dy;
        return p;
    }

    std::vector<geometry_msgs::msg::Point> HermitePathGenerator::getRightBounds(hermite_path_msgs::msg::HermitePath path,int resolution)
    {
        std::vector<geometry_msgs::msg::Point> points;
        double step_size = 1.0/(double)resolution;
        for(int i=0;i<(resolution+1);i++)
        {
            double t = step_size * (double)i;
            geometry_msgs::msg::Vector3 vec = getNormalVector(path,t);
            double theta = std::atan2(vec.y,vec.x);
            geometry_msgs::msg::Point p = getPointOnHermitePath(path,t);
            geometry_msgs::msg::Point point;
            point.x = p.x + robot_width_*std::cos(theta);
            point.y = p.y + robot_width_*std::sin(theta);
            points.push_back(point);
        }
        return points;
    }

    geometry_msgs::msg::Vector3 HermitePathGenerator::getVectorFromPose(geometry_msgs::msg::Pose pose,double magnitude)
    {
        geometry_msgs::msg::Vector3 dir = quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
        geometry_msgs::msg::Vector3 vector;
        vector.x = magnitude * std::cos(dir.z);
        vector.y = magnitude * std::sin(dir.z);
        vector.z = 0;
        return vector;
    }

    geometry_msgs::msg::Vector3 HermitePathGenerator::getNormalVector(hermite_path_msgs::msg::HermitePath path,double t)
    {
        geometry_msgs::msg::Vector3 vec;
        vec.x = 3*path.ay*t*t + 2*path.by*t + path.cy;
        vec.y = (3*path.ax*t*t + 2*path.bx*t + path.cx)*-1;
        return vec;
    }

    visualization_msgs::msg::MarkerArray HermitePathGenerator::generateMarker(hermite_path_msgs::msg::HermitePathStamped path,int resolution)
    {
        visualization_msgs::msg::MarkerArray marker;

        // Setup Color
        std_msgs::msg::ColorRGBA color_center_line;
        color_center_line.r = 1.0;
        color_center_line.g = 0.0;
        color_center_line.b = 0.0;
        color_center_line.a = 1.0;

        std_msgs::msg::ColorRGBA color_bounds;
        color_bounds.r = 0.0;
        color_bounds.g = 0.0;
        color_bounds.b = 1.0;
        color_bounds.a = 1.0;
        
        // Setup Center Line Marker
        visualization_msgs::msg::Marker center_line;
        center_line.header = path.header;
        center_line.ns = "center_line";
        center_line.id = 0;
        center_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        center_line.action = visualization_msgs::msg::Marker::ADD;
        center_line.frame_locked = false;
        center_line.scale.x = 0.1;
        center_line.points = getPointsOnHermitePath(path.path,resolution);
        center_line.colors = std::vector<std_msgs::msg::ColorRGBA>(center_line.points.size(),color_center_line);
        marker.markers.push_back(center_line);

        //Setup Left Bounds Marker
        /*
        visualization_msgs::msg::Marker left_bounds;
        left_bounds.header = path.header;
        left_bounds.ns = "left_bounds";
        left_bounds.id = 0;
        left_bounds.type = visualization_msgs::msg::Marker::LINE_STRIP;
        left_bounds.action = visualization_msgs::msg::Marker::ADD;
        left_bounds.frame_locked = false;
        left_bounds.scale.x = 0.1;
        left_bounds.points = getLeftBounds(path.path,resolution);
        left_bounds.colors = std::vector<std_msgs::msg::ColorRGBA>(left_bounds.points.size(),color_bounds);
        marker.markers.push_back(left_bounds);
        */

        //Setup Right Bounds Marker
        /*
        visualization_msgs::msg::Marker right_bounds;
        right_bounds.header = path.header;
        right_bounds.ns = "right_bounds";
        right_bounds.id = 0;
        right_bounds.type = visualization_msgs::msg::Marker::LINE_STRIP;
        right_bounds.action = visualization_msgs::msg::Marker::ADD;
        right_bounds.frame_locked = false;
        right_bounds.scale.x = 0.1;
        right_bounds.points = getRightBounds(path.path,resolution);
        right_bounds.colors = std::vector<std_msgs::msg::ColorRGBA>(right_bounds.points.size(),color_bounds);
        marker.markers.push_back(right_bounds);
        */

        return marker;
    }
}