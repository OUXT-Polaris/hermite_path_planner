#ifndef VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_H_INCLUDED
#define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_BUILDING_DLL
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT
    #else
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT
    #endif
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC_TYPE VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL
#else
    #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_LOCAL
    #endif
    #define VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <hermite_path_planner/hermite_path_generator.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/optional.hpp>

namespace velocity_planner
{
    class VelocityPlannerComponent: public rclcpp::Node
    {
    public:
        VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_PUBLIC
        explicit VelocityPlannerComponent(const rclcpp::NodeOptions & options);
    private:
        rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
        void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
        boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_twist_sub_;
        boost::optional<geometry_msgs::msg::Twist> current_twist_;
        void currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
        boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;
        void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        void updatePath();
        std::mutex mtx_;
        std::shared_ptr<hermite_path_planner::HermitePathGenerator> generator_;
        double maximum_accerelation_ = 0.3;
        double minimum_deceleration_ = -0.1;
    };
}

#endif  //VELOCITY_PLANNER_VELOCITY_PLANNER_COMPONENT_H_INCLUDED