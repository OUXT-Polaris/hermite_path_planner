#ifndef VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_H_INCLUDED
#define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT __attribute__ ((dllexport))
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT __attribute__ ((dllimport))
    #else
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT __declspec(dllexport)
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT __declspec(dllimport)
    #endif
    #ifdef VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_BUILDING_DLL
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT
    #else
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT
    #endif
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC_TYPE VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL
#else
    #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_EXPORT __attribute__ ((visibility("default")))
    #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_IMPORT
    #if __GNUC__ >= 4
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
    #else
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
        #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_LOCAL
    #endif
    #define VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/exact_time.h>

namespace velocity_planner
{
    typedef hermite_path_msgs::msg::HermitePathStamped HermitePathStamped;
    typedef message_filters::Subscriber<HermitePathStamped> HermitePathSubscriber;
    typedef message_filters::sync_policies::ExactTime
      <HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped, 
      HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped> SyncPolicy;

    class PlannerConcatenatorComponent: public rclcpp::Node
    {
    public:
        VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_PUBLIC
        explicit PlannerConcatenatorComponent(const rclcpp::NodeOptions & options);
    private:
        std::array<std::shared_ptr<HermitePathSubscriber>,8> sub_ptrs_;
        rclcpp::Publisher<hermite_path_msgs::msg::HermitePathStamped>::SharedPtr hermite_path_pub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
        message_filters::PassThrough<HermitePathStamped> nf_;
        int num_input_;
        std::array<std::string,8> input_topics_;
        void callback(const HermitePathStamped::SharedPtr &in0, const HermitePathStamped::SharedPtr &in1,
            const HermitePathStamped::SharedPtr &in2, const HermitePathStamped::SharedPtr &in3,
            const HermitePathStamped::SharedPtr &in4, const HermitePathStamped::SharedPtr &in5,
            const HermitePathStamped::SharedPtr &in6, const HermitePathStamped::SharedPtr &in7);
    };
}

#endif  //VELOCITY_PLANNER_PLANNER_CONCATENATOR_COMPONENT_H_INCLUDED