#include <velocity_planner/planner_concatenator_component.h>

namespace velocity_planner
{
    PlannerConcatenatorComponent::PlannerConcatenatorComponent(const rclcpp::NodeOptions & options)
    : Node("planner_concatenator", options)
    {
        hermite_path_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>
            ("~/hermite_path", 1);
        declare_parameter("num_input",2);
        get_parameter("num_input",num_input_);
        assert(num_input_>=2 && num_input_<=8);
        for(int i=0; i<num_input_; i++)
        {
            declare_parameter("input_topic"+std::to_string(i),
                get_name() + std::string("/input") + std::to_string(i));
            get_parameter("input_topic"+std::to_string(i),input_topics_[i]);
            std::shared_ptr<HermitePathSubscriber> sub_ptr(new HermitePathSubscriber(this,input_topics_[i]));
            sub_ptrs_[i] = sub_ptr;
        }
        if(num_input_ == 2)
        {
            sync2_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped> >(*sub_ptrs_[0],*sub_ptrs_[1],10);
            sync2_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback2, this, 
                std::placeholders::_1, std::placeholders::_2));
        }
        if(num_input_ == 3)
        {
            sync3_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped> >(*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],10);
            sync3_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback3, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        }
        if(num_input_ == 4)
        {
            sync4_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped, HermitePathStamped> >
                (*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],*sub_ptrs_[3], 10);
            sync4_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback4, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        }
        if(num_input_ == 5)
        {
            sync5_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped, 
                    HermitePathStamped, HermitePathStamped> >
                (*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],*sub_ptrs_[3], *sub_ptrs_[4], 10);
            sync5_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback5, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                    std::placeholders::_4, std::placeholders::_5));
        }
        if(num_input_ == 6)
        {
            sync6_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped, 
                    HermitePathStamped, HermitePathStamped, HermitePathStamped> >
                (*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],*sub_ptrs_[3], *sub_ptrs_[4], 
                    *sub_ptrs_[5], 10);
            sync6_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback6, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                    std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
        }
        if(num_input_ == 7)
        {
            sync7_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped, 
                    HermitePathStamped, HermitePathStamped, HermitePathStamped,
                    HermitePathStamped> >
                (*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],*sub_ptrs_[3], *sub_ptrs_[4], 
                    *sub_ptrs_[5], *sub_ptrs_[6], 10);
            sync7_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback7, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                    std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                    std::placeholders::_7));
        }
        if(num_input_ == 8)
        {
            sync8_ = std::make_shared<message_filters::TimeSynchronizer
                <HermitePathStamped, HermitePathStamped, HermitePathStamped, 
                    HermitePathStamped, HermitePathStamped, HermitePathStamped,
                    HermitePathStamped, HermitePathStamped> >
                (*sub_ptrs_[0],*sub_ptrs_[1],*sub_ptrs_[2],*sub_ptrs_[3], *sub_ptrs_[4], 
                    *sub_ptrs_[5], *sub_ptrs_[6], *sub_ptrs_[7], 10);
            sync8_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback8, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
                    std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
                    std::placeholders::_7, std::placeholders::_8));
        }
    }

    void PlannerConcatenatorComponent::callback2(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1)
    {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
    }

    void PlannerConcatenatorComponent::callback3(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }

    void PlannerConcatenatorComponent::callback4(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v3 = in3->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r3));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }

    void PlannerConcatenatorComponent::callback5(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
        const HermitePathStamped::ConstSharedPtr in4)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v3 = in3->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v4 = in4->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r4));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }

    void PlannerConcatenatorComponent::callback6(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
        const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v3 = in3->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v4 = in4->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v5 = in5->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v5));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r5));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }

    void PlannerConcatenatorComponent::callback7(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
        const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
        const HermitePathStamped::ConstSharedPtr in6)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v3 = in3->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v4 = in4->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v5 = in5->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v5));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r5));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v6 = in6->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v6));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r6 = in6->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r6));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }

    void PlannerConcatenatorComponent::callback8(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
        const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
        const HermitePathStamped::ConstSharedPtr in6, const HermitePathStamped::ConstSharedPtr in7)
        {
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;

            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v2 = in2->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r2 = in2->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r2));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v3 = in3->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r3 = in3->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r3));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v4 = in4->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r4 = in4->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r4));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v5 = in5->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v5));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r5 = in5->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r5));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v6 = in6->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v6));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r6 = in6->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r6));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> v7 = in7->target_velocity;
            std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v7));
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> r7 = in7->reference_velocity;
            std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r7));

            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }
}