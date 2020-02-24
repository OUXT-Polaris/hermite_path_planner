#include <hermite_path_planner/hermite_path_generator.h>
#include <iostream>

namespace hermite_path_planner
{
    HermitePathGenerator::HermitePathGenerator(double robot_width)
    {
        robot_width_ = robot_width;
    }

    boost::optional<double> HermitePathGenerator::getDistanceInFrenetCoordinate(
        hermite_path_msgs::msg::HermitePath path,geometry_msgs::msg::Point p)
    {
        double coefficients[6];
        coefficients[0] = path.cy*(p.x-path.dx) +path.cx*(p.y-path.dy);
        coefficients[1] = 2*path.by*(p.x-path.dx) +2*path.bx*(p.y-path.dy);
        coefficients[2] = 3*path.ay*(p.x-path.dx) -2*path.by*path.cx +3*path.ax*(p.y-path.dy) -2*path.bx*path.cy;
        coefficients[3] = -3*path.ax*path.cy -2*path.by*path.bx -path.cx*path.ay 
            - 3*path.ay*path.cx -2*path.bx*path.by -path.cy*path.ax;
        coefficients[4] = -3*path.ay*path.bx -3*path.ax*path.by;
        coefficients[5] = -6*path.ay*path.ax;
        auto func = [](double coefficients[6],double t)
        {
            double t5 = std::pow(t,5);
            double t4 = std::pow(t,4);
            double t3 = std::pow(t,3);
            double t2 = std::pow(t,2);
            double a = t5*coefficients[5] + t4*coefficients[4] + 
                t3*coefficients[3] + t2*coefficients[2] + 
                t*coefficients[1] + coefficients[0];
            return a;
        };
        
        auto get_newton_step_size_func = [](double coefficients[6],double t)
        {
            double t5 = std::pow(t,5);
            double t4 = std::pow(t,4);
            double t3 = std::pow(t,3);
            double t2 = std::pow(t,2);
            double a = t5*coefficients[5] + t4*coefficients[4] + 
                t3*coefficients[3] + t2*coefficients[2] + 
                t*coefficients[1] + coefficients[0];
            double b = 5*t4*coefficients[5] + 4*t3*coefficients[4] + 
                3*t2*coefficients[3] + 2*t*coefficients[2]+coefficients[1];
            return -1*b/a;
	    };

        constexpr int initial_resolution = 30;
        constexpr int max_iteration = 30;
        constexpr double torelance = 0.001;

        bool initial_point_finded = false;
        double step_size = (double)1.0/(double)initial_resolution;
        double ret = 0.0;
        std::array<double,initial_resolution> initial_value_candidates;
        std::array<double,initial_resolution> errors;
        for(int i=0; i<initial_resolution; i++)
        {
            initial_value_candidates[i] = (0.5+(double)i)*step_size;
            errors[i] = func(coefficients,initial_value_candidates[i]);
        }
        for(int i=0; i<initial_resolution-1; i++)
        {
            if((errors[i]*errors[i+1])<0)
            {
                initial_point_finded = true;
                ret = (initial_value_candidates[i]+initial_value_candidates[i+1])/2;
                break;
            }
        }
        if(!initial_point_finded)
        {
            return boost::none;
        }
        for(int i=0; i<max_iteration; i++)
        {
            double error = func(coefficients,ret);
            if(std::fabs(error)<torelance)
            {
                return ret;
            }
            ret = ret + get_newton_step_size_func(coefficients,ret);
        }
        return ret;
    }

    double HermitePathGenerator::getLength(hermite_path_msgs::msg::HermitePath path,int resolution)
    {
        double ret = 0.0;
        std::vector<geometry_msgs::msg::Point> points = getPointsOnHermitePath(path,resolution);
        for(int i=0; i<(resolution-1); i++) 
        {
            double length_segment = std::sqrt(std::pow(points[i].x-points[i+1].x,2)+std::pow(points[i].y-points[i+1].y,2));
            ret = length_segment + ret;
        }
        return ret;
    }

    double HermitePathGenerator::calculateNewtonMethodStepSize(hermite_path_msgs::msg::HermitePath path,
        geometry_msgs::msg::Point center,double radius,double t)
    {
        double t3 = t*t*t;
        double t2 = t*t;
        double f = std::pow((path.ax*t3 + path.bx*t2 + path.cx*t + path.dx - center.x),2) 
            + std::pow((path.ay*t3 + path.by*t2 + path.cy*t + path.dy - center.y),2) - radius*radius;
        double term_x = 2 * (path.ax*t3 + path.bx*t2 + path.cx*t + path.dx - center.x) * (3*path.ax*t3 + 2*path.bx*t2 + path.cx);
        double term_y = 2 * (path.ay*t3 + path.by*t2 + path.cy*t + path.dy - center.y) * (3*path.ay*t3 + 2*path.by*t2 + path.cy);
        //std::cout << term_x << std::endl;
        return f/(term_x+term_y);
    }

    boost::optional<double> HermitePathGenerator::checkFirstCollisionWithCircle(hermite_path_msgs::msg::HermitePath path,
        geometry_msgs::msg::Point center,double radius)
    {
        constexpr int initial_resolution = 30;
        constexpr int max_iteration = 30;
        constexpr double torelance = 0.001;
        double step_size = 1.0/(double)initial_resolution;
        std::vector<geometry_msgs::msg::Point> points_on_path = getPointsOnHermitePath(path,initial_resolution);
        std::array<double,initial_resolution> errors;
        for(int i=0; i<initial_resolution; i++)
        {
            errors[i] = std::sqrt(std::pow(points_on_path[i].x-center.x,2)+std::pow(points_on_path[i].y-center.y,2))-radius;
        }
        double ret;
        bool initial_point_finded = false;
        for(int i=0; i<initial_resolution-1; i++)
        {
            if((errors[i]*errors[i+1])<0)
            {
                ret = step_size*(double)(2*i+1)/2.0;
                initial_point_finded = true;
                break;
            }
        }
        if(!initial_point_finded)
        {
            return boost::none;
        }
        for(int i=0; i<max_iteration; i++)
        {
            geometry_msgs::msg::Point p = getPointOnHermitePath(path,ret);
            double error = std::sqrt(std::pow(p.x-center.x,2)+std::pow(p.y-center.y,2)) - radius;
            if(std::fabs(error)<torelance)
            {
                return ret;
            }
            double diff = calculateNewtonMethodStepSize(path,center,radius,ret);
            ret = ret - diff;
        }
        return ret;
    }

    hermite_path_msgs::msg::HermitePath HermitePathGenerator::generateHermitePath(geometry_msgs::msg::Pose start,geometry_msgs::msg::Pose goal)
    {
        hermite_path_msgs::msg::HermitePath path;
        geometry_msgs::msg::Vector3 start_vector = getVectorFromPose(start,50.0);
        geometry_msgs::msg::Vector3 goal_vector = getVectorFromPose(goal,50.0);
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
            point.x = p.x - 0.5*robot_width_*std::cos(theta);
            point.y = p.y - 0.5*robot_width_*std::sin(theta);
            points.push_back(point);
        }
        return points;
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
            point.x = p.x + 0.5*robot_width_*std::cos(theta);
            point.y = p.y + 0.5*robot_width_*std::sin(theta);
            points.push_back(point);
        }
        return points;
    }

    visualization_msgs::msg::Marker HermitePathGenerator::getBoundsPolygon(hermite_path_msgs::msg::HermitePathStamped path,int resolution,double z_offset)
    {
        std_msgs::msg::ColorRGBA color;
        color.r = 0.2;
        color.g = 0.8;
        color.b = 0.8;
        color.a = 0.5;
        visualization_msgs::msg::Marker marker;
        marker.header = path.header;
        marker.ns = "polygon";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        std::vector<geometry_msgs::msg::Point> left_bounds = getLeftBounds(path.path,resolution);
        std::vector<geometry_msgs::msg::Point> right_bounds = getRightBounds(path.path,resolution);
        int num_sections = left_bounds.size()-1;
        assert(left_bounds.size()==right_bounds.size());
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = color;
        for(int i=0; i<num_sections; i++)
        {
            geometry_msgs::msg::Point pr_0 = right_bounds[i];
            pr_0.z = z_offset;
            geometry_msgs::msg::Point pl_0 = left_bounds[i];
            pl_0.z = z_offset;
            geometry_msgs::msg::Point pr_1 = right_bounds[i+1];
            pr_1.z = z_offset;
            geometry_msgs::msg::Point pl_1 = left_bounds[i+1];
            pl_1.z = z_offset;
            marker.points.push_back(pr_0);
            marker.points.push_back(pl_0);
            marker.points.push_back(pr_1);
            marker.colors.push_back(color);
            marker.points.push_back(pl_0);
            marker.points.push_back(pl_1);
            marker.points.push_back(pr_1);
            marker.colors.push_back(color);
        }
        return marker;
    }

    geometry_msgs::msg::Point HermitePathGenerator::getPointOnHermitePath(hermite_path_msgs::msg::HermitePath path,double t)
    {
        geometry_msgs::msg::Point p;
        p.z = 0;
        p.x = path.ax*std::pow(t,3) + path.bx*std::pow(t,2) + path.cx*t + path.dx;
        p.y = path.ay*std::pow(t,3) + path.by*std::pow(t,2) + path.cy*t + path.dy;
        return p;
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

    geometry_msgs::msg::Vector3 HermitePathGenerator::getTangentVector(hermite_path_msgs::msg::HermitePath path,double t)
    {
        geometry_msgs::msg::Vector3 vec;
        vec.x = 3*path.ax*t*t + 2*path.bx*t + path.cx;
        vec.y = 3*path.ay*t*t + 2*path.by*t + path.cy;
        return vec;
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
        marker.markers.push_back(getBoundsPolygon(path,resolution,-0.3));
        return marker;
    }
}