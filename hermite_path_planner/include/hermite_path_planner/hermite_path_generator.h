#ifndef HERMITE_PATH_PLANNER_HERMITE_PATH_GENERATOR_H_INCLUDED
#define HERMITE_PATH_PLANNER_HERMITE_PATH_GENERATOR_H_INCLUDED

#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <quaternion_operation/quaternion_operation.h>

namespace hermite_path_planner
{
    class HermitePathGenerator
    {
    public:
        HermitePathGenerator(double robot_width);
        hermite_path_msgs::msg::HermitePath generateHermitePath(geometry_msgs::msg::Pose start,geometry_msgs::msg::Pose goal);
        geometry_msgs::msg::Point getPointOnHermitePath(hermite_path_msgs::msg::HermitePath path,double t);
        std::vector<geometry_msgs::msg::Point> getPointsOnHermitePath(hermite_path_msgs::msg::HermitePath path,int resolution);
        visualization_msgs::msg::MarkerArray generateMarker(hermite_path_msgs::msg::HermitePathStamped path,int resolution);
    private:
        visualization_msgs::msg::Marker getBoundsPolygon(hermite_path_msgs::msg::HermitePathStamped path,int resolution,double z_offset);
        geometry_msgs::msg::Vector3 getVectorFromPose(geometry_msgs::msg::Pose pose,double magnitude);
        geometry_msgs::msg::Vector3 getTangentVector(hermite_path_msgs::msg::HermitePath path,double t);
        geometry_msgs::msg::Vector3 getNormalVector(hermite_path_msgs::msg::HermitePath path,double t);
        std::vector<geometry_msgs::msg::Point> getLeftBounds(hermite_path_msgs::msg::HermitePath path,int resolution);
        std::vector<geometry_msgs::msg::Point> getRightBounds(hermite_path_msgs::msg::HermitePath path,int resolution);
        double robot_width_;
    };
}

#endif  //HERMITE_PATH_PLANNER_HERMITE_PATH_GENERATOR_H_INCLUDED