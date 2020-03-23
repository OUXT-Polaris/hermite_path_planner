#ifndef VELOCITY_PLANNER_OBSTACLE_PLANNER_H_INCLUDED
#define VELOCITY_PLANNER_OBSTACLE_PLANNER_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_BUILDING_DLL
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT
    #else
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT
    #endif
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC_TYPE VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL
#else
    #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_LOCAL
    #endif
    #define VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <boost/optional.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace velocity_planner
{
    class ObstaclePlannerComponent: public rclcpp::Node
    {
    public:
        VELOCITY_PLANNER_OBSTACLE_PLANNER_COMPONENT_PUBLIC
        explicit ObstaclePlannerComponent(const rclcpp::NodeOptions & options);
    private:
        rclcpp::Subscription<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_sub_;
        void hermitePathCallback(const hermite_path_msgs::msg::HermitePathStamped::SharedPtr data);
        boost::optional<hermite_path_msgs::msg::HermitePathStamped> path_;
        rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
        rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr replan_path_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data);
    };
}

#endif //VELOCITY_PLANNER_OBSTACLE_PLANNER_H_INCLUDED