#ifndef PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_H_INCLUDED
#define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT __declspec(dllexport)
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_BUILDING_DLL
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT
    #else
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT
    #endif
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC_TYPE PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL
#else
    #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
        #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_LOCAL
    #endif
    #define PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <hermite_path_planner/hermite_path_generator.h>

namespace pure_pursuit_planner
{
    class PurePursuitPlannerComponent : public rclcpp::Node
    {
    public:
        PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_PUBLIC
        explicit PurePursuitPlannerComponent(const rclcpp::NodeOptions & options);
    };
}

#endif  //PURE_PURSUIT_PLANNER_PURE_PURSUIT_PLANNER_COMPONENT_H_INCLUDED