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

namespace hermite_path_planner
{
    class HermitePathPlannerComponent : public rclcpp::Node
    {
    public:
        HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_PUBLIC
        explicit HermitePathPlannerComponent(const rclcpp::NodeOptions & options);
    };
}

#endif  //HERMITE_PATH_PLANNER_HERMITE_PATH_PLANNER_COMPONENT_H_INCLUDED