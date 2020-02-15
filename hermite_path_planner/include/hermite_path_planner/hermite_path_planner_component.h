#ifndef HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_H_INCLUDED
#define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_BUILDING_DLL
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_EXPORT
    #else
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_IMPORT
    #endif
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC_TYPE HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_LOCAL
#else
    #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC
        #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_LOCAL
    #endif
    #define HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <quaternion_operation/quaternion_operation.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/optional.hpp>

namespace hermite_path_planner
{
    class HermitePathPlannerComponent : public rclcpp::Node
    {
    public:
        HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC
        explicit HermitePathPlannerComponent(const rclcpp::NodeOptions & options);
    private:
        geometry_msgs::msg::PoseStamped TransformToPlanningFrame(geometry_msgs::msg::PoseStamped pose);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
        void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
        void CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
        std::string planning_frame_id_;
        std::string goal_pose_topic_;
        std::string current_pose_topic_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        hermite_path_msgs::msg::HermitePath generateHermitePath(geometry_msgs::msg::Pose start,geometry_msgs::msg::Pose goal);
        geometry_msgs::msg::Vector3 getVectorFromPose(geometry_msgs::msg::Pose pose,double magnitude);
        visualization_msgs::msg::MarkerArray generateMarker(hermite_path_msgs::msg::HermitePathStamped path,int resolution);
        geometry_msgs::msg::Point getPointOnHermitePath(hermite_path_msgs::msg::HermitePath path,double t);
    };
}

#endif  //HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_H_INCLUDED